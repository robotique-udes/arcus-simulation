import os
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import messagebox


BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class PrepareMapApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Prepare Map")
        self.root.geometry("420x560")
        self.root.resizable(False, False)

        self.running = False
        self.buttons = []

        header = tk.Label(
            root,
            text="Map Preparation",
            font=("Helvetica", 16, "bold"),
            pady=10,
        )
        header.pack()

        self.status_var = tk.StringVar(value="Ready")
        status = tk.Label(root, textvariable=self.status_var, fg="#1f4e79")
        status.pack(pady=(0, 8))

        actions = [
            ("Import maps", self._run_import_maps),
            ("Clean map", self._run_clean_map),
            ("Generate raceline", self._run_generate_raceline),
            ("Generate speed zones", self._run_generate_speed_zones),
            ("Generate algo zones", self._run_generate_algo_zones),
            ("Export raceline", self._run_export_raceline),
            ("Export speed zones", self._run_export_speed_zones),
            ("Export algos", self._run_export_algos),
        ]

        button_frame = tk.Frame(root, padx=20, pady=8)
        button_frame.pack(fill=tk.BOTH, expand=True)

        for label, callback in actions:
            btn = tk.Button(
                button_frame,
                text=label,
                command=callback,
                height=2,
                font=("Helvetica", 11),
            )
            btn.pack(fill=tk.X, pady=5)
            self.buttons.append(btn)

    def _set_running(self, running, message=None):
        self.running = running
        state = tk.DISABLED if running else tk.NORMAL
        for btn in self.buttons:
            btn.configure(state=state)
        if message is not None:
            self.status_var.set(message)

    def _run_command_async(self, label, command):
        if self.running:
            return

        self._set_running(True, f"Running: {label}")

        def worker():
            try:
                subprocess.run(command, cwd=BASE_DIR, check=True)
            except subprocess.CalledProcessError as exc:
                details = f"Command failed with exit code {exc.returncode}."
                self.root.after(
                    0,
                    lambda: self._finish_with_error(label, details),
                )
                return
            except Exception as exc:  # pragma: no cover
                details = str(exc)
                self.root.after(0, lambda: self._finish_with_error(label, details))
                return

            self.root.after(0, lambda: self._finish_success(label))

        threading.Thread(target=worker, daemon=True).start()

    def _finish_success(self, label):
        self._set_running(False, f"Finished: {label}")
        messagebox.showinfo("Done", f"{label} finished. Returned to main GUI.")

    def _finish_with_error(self, label, details):
        self._set_running(False, f"Failed: {label}")
        messagebox.showerror("Error", f"{label} failed.\n\n{details}")

    def _run_import_maps(self):
        script = os.path.join("utils", "importMaps.sh")
        self._run_command_async("Import maps", ["bash", script])

    def _run_clean_map(self):
        self._run_command_async("Clean map", [sys.executable, "map_cleaner.py"])

    def _run_generate_raceline(self):
        self._run_command_async("Generate raceline", [sys.executable, "generate_raceline.py"])

    def _run_generate_speed_zones(self):
        self._run_command_async("Generate speed zones", [sys.executable, "generate_speed_zones.py"])

    def _run_generate_algo_zones(self):
        self._run_command_async("Generate algo zones", [sys.executable, "generate_algo_zones.py"])

    def _run_export_raceline(self):
        script = os.path.join("utils", "exportRaceline.sh")
        self._run_command_async("Export raceline", ["bash", script])

    def _run_export_speed_zones(self):
        script = os.path.join("utils", "exportSpeedZones.sh")
        self._run_command_async("Export speed zones", ["bash", script])

    def _run_export_algos(self):
        script = os.path.join("utils", "exportAlgos.sh")
        self._run_command_async("Export algos", ["bash", script])


def main():
    root = tk.Tk()
    PrepareMapApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
