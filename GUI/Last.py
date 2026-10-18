import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation


class PIDControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 WiFi PID Motor Controller")
        self.root.geometry("1050x650")

        self.max_points    = 100
        self.x_data        = deque(maxlen=self.max_points)
        self.setpoint_data = deque(maxlen=self.max_points)
        self.measured_data = deque(maxlen=self.max_points)
        self.pwm_data      = deque(maxlen=self.max_points)
        self.time_counter  = 0

        self.sock       = None
        self.is_reading = False
        self.esp32_port = 8080
        self.motor_on   = False

        self.setup_ui()
        self.setup_plot()

    def setup_ui(self):
        f = ttk.Frame(self.root, padding="10")
        f.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(f, text="ESP32 IP Address:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.ent_ip = ttk.Entry(f, width=15)
        self.ent_ip.insert(0, "10.51.138.18")
        self.ent_ip.grid(row=0, column=1, pady=5)

        self.btn_connect = ttk.Button(f, text="Connect via WiFi",
                                      command=self.toggle_connection)
        self.btn_connect.grid(row=1, column=0, columnspan=2, pady=10, sticky="ew")

        ttk.Separator(f, orient='horizontal').grid(
            row=2, column=0, columnspan=2, sticky='ew', pady=10)

        self.btn_motor = tk.Button(
            f, text="START MOTOR", bg="green", fg="white",
            font=("Arial", 10, "bold"), command=self.toggle_motor, state=tk.DISABLED
        )
        self.btn_motor.grid(row=3, column=0, columnspan=2, pady=5, sticky="ew")

        ttk.Separator(f, orient='horizontal').grid(
            row=4, column=0, columnspan=2, sticky='ew', pady=10)

        # UPDATED: New default detuned parameters
        fields = [
            ("Target RPM:", "ent_rpm", "100.0"),
            ("Kp:",         "ent_kp",  "0.3"),
            ("Ki:",         "ent_ki",  "0.1"),
            ("Kd:",         "ent_kd",  "0.02"),
        ]
        for i, (lbl, attr, default) in enumerate(fields):
            ttk.Label(f, text=lbl).grid(row=5 + i, column=0, sticky=tk.W, pady=4)
            e = ttk.Entry(f, width=10)
            e.insert(0, default)
            e.grid(row=5 + i, column=1, pady=4)
            setattr(self, attr, e)

        self.btn_send = ttk.Button(
            f, text="Update PID Parameters",
            command=self.send_parameters, state=tk.DISABLED
        )
        self.btn_send.grid(row=9, column=0, columnspan=2, pady=15, sticky="ew")

        self.lbl_status = ttk.Label(f, text="Disconnected", foreground="red")
        self.lbl_status.grid(row=10, column=0, columnspan=2, pady=5)

        ttk.Separator(f, orient='horizontal').grid(
            row=11, column=0, columnspan=2, sticky='ew', pady=10)

        self.lbl_actual_rpm = ttk.Label(
            f, text="Actual RPM: --", font=("Arial", 14, "bold"), foreground="blue")
        self.lbl_actual_rpm.grid(row=12, column=0, columnspan=2, pady=5)

        self.lbl_pwm = ttk.Label(f, text="PWM: --", font=("Arial", 11))
        self.lbl_pwm.grid(row=13, column=0, columnspan=2, pady=2)

        self.lbl_sp = ttk.Label(f, text="Setpoint: --", font=("Arial", 11))
        self.lbl_sp.grid(row=14, column=0, columnspan=2, pady=2)

        self.lbl_err = ttk.Label(f, text="Error: --", font=("Arial", 11))
        self.lbl_err.grid(row=15, column=0, columnspan=2, pady=2)

        self.lbl_mode = ttk.Label(f, text="Mode: --", font=("Arial", 11, "bold"))
        self.lbl_mode.grid(row=16, column=0, columnspan=2, pady=2)

    def setup_plot(self):
        plot_frame = ttk.Frame(self.root)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig, (self.ax1, self.ax2) = plt.subplots(
            2, 1, figsize=(7, 5), gridspec_kw={'height_ratios': [2, 1]}
        )
        self.fig.tight_layout(pad=3.0)

        self.line_setpoint, = self.ax1.plot(
            [], [], label='Setpoint RPM', color='red', linestyle='--')
        self.line_measured, = self.ax1.plot(
            [], [], label='Measured RPM', color='blue')
        self.ax1.set_title("Motor Speed (RPM)")
        self.ax1.set_ylabel("RPM")
        self.ax1.set_ylim(0, 350)
        self.ax1.legend(loc="upper left")
        self.ax1.grid(True)

        self.line_pwm, = self.ax2.plot(
            [], [], label='PWM Output', color='green')
        self.ax2.set_title("Controller Output (PWM)")
        self.ax2.set_ylabel("PWM")
        self.ax2.set_xlabel("Time (samples)")
        self.ax2.set_ylim(-20, 270)
        self.ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.8)
        self.ax2.legend(loc="upper left")
        self.ax2.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100,
            blit=False, cache_frame_data=False
        )

    def toggle_motor(self):
        if not self.sock:
            return
        self.motor_on = not self.motor_on
        cmd = "START\n" if self.motor_on else "STOP\n"
        self.btn_motor.config(
            text="STOP MOTOR"  if self.motor_on else "START MOTOR",
            bg  ="red"         if self.motor_on else "green"
        )
        try:
            self.sock.sendall(cmd.encode('utf-8'))
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def toggle_connection(self):
        if self.is_reading:
            self.is_reading = False
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
            self._reset_ui()
        else:
            ip = self.ent_ip.get().strip()
            if not ip:
                messagebox.showerror("Error", "Enter the ESP32 IP address.")
                return
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3.0)
                self.sock.connect((ip, self.esp32_port))
                self.sock.settimeout(None)
                self.is_reading = True
                self.btn_connect.config(text="Disconnect")
                self.btn_send.config(state=tk.NORMAL)
                self.btn_motor.config(state=tk.NORMAL)
                self.lbl_status.config(
                    text=f"Connected to {ip}", foreground="green")
                threading.Thread(
                    target=self.read_wifi_data, daemon=True).start()
            except Exception as e:
                messagebox.showerror("Connection Error", f"Could not connect.\n{e}")
                self.lbl_status.config(text="Connection Failed", foreground="red")

    def send_parameters(self):
        if not (self.sock and self.is_reading):
            return
        try:
            rpm = float(self.ent_rpm.get())
            kp  = float(self.ent_kp.get())
            ki  = float(self.ent_ki.get())
            kd  = float(self.ent_kd.get())
        except ValueError:
            messagebox.showerror("Input Error", "All fields must be valid numbers.")
            return
        try:
            self.sock.sendall(f"{rpm},{kp},{ki},{kd}\n".encode('utf-8'))
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def read_wifi_data(self):
        buffer = ""
        while self.is_reading and self.sock:
            try:
                chunk = self.sock.recv(1024).decode('utf-8')
                if not chunk:
                    break
                buffer += chunk
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if "Setpoint:" in line and "Measured:" in line and "PWM:" in line:
                        try:
                            parts = line.split(',')
                            sp   = float(parts[0].split(':')[1])
                            meas = float(parts[1].split(':')[1])
                            pwm  = float(parts[2].split(':')[1])
                            mode = parts[3].split(':')[1] if len(parts) > 3 else "?"
                            err  = sp - meas

                            self.time_counter += 1
                            self.x_data.append(self.time_counter)
                            self.setpoint_data.append(sp)
                            self.measured_data.append(meas)
                            self.pwm_data.append(pwm)

                            mode_color = {"Drive": "green", "Coast": "gray",
                                          "Off": "black"}.get(mode, "black")

                            self.root.after(0, lambda m=meas, p=pwm, s=sp,
                                            er=err, mt=mode, mc=mode_color: (
                                self.lbl_actual_rpm.config(text=f"Actual RPM: {m:.1f}"),
                                self.lbl_pwm.config(text=f"PWM: {int(p)}"),
                                self.lbl_sp.config(text=f"Setpoint: {s:.1f}"),
                                self.lbl_err.config(
                                    text=f"Error: {er:.1f} RPM",
                                    foreground="red" if abs(er) > 10 else "green"),
                                self.lbl_mode.config(text=f"Mode: {mt}",
                                                     foreground=mc),
                            ))
                        except (IndexError, ValueError):
                            pass
            except Exception:
                break

        self.root.after(0, self._on_disconnect)

    def _on_disconnect(self):
        self.is_reading = False
        self.sock       = None
        self._reset_ui()
        self.lbl_status.config(
            text="Disconnected (connection lost)", foreground="red")

    def _reset_ui(self):
        self.motor_on = False
        self.btn_connect.config(text="Connect via WiFi")
        self.btn_send.config(state=tk.DISABLED)
        self.btn_motor.config(state=tk.DISABLED, text="START MOTOR", bg="green")
        self.lbl_status.config(text="Disconnected", foreground="red")

    def update_plot(self, frame):
        if not self.x_data:
            return
        x = list(self.x_data)
        self.line_setpoint.set_data(x, list(self.setpoint_data))
        self.line_measured.set_data(x, list(self.measured_data))
        self.line_pwm.set_data(x, list(self.pwm_data))
        x_min = max(0, self.time_counter - self.max_points)
        x_max = self.time_counter + 5
        self.ax1.set_xlim(x_min, x_max)
        self.ax2.set_xlim(x_min, x_max)


if __name__ == "__main__":
    root = tk.Tk()
    app  = PIDControlApp(root)
    root.mainloop()