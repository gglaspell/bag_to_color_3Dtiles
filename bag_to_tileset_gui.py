#!/usr/bin/env python3
"""
bag_to_tileset_gui.py — Tkinter GUI for the bag-to-tileset Docker container.
Builds and runs the docker run command with all supported parameters,
including the RGB camera colorization options.
"""
import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext, messagebox
import subprocess
import threading
import shlex
from pathlib import Path

# ── colour tokens ─────────────────────────────────────────────────────────────
BG       = "#1e1e2e"
SURFACE  = "#252538"
SURFACE2 = "#2e2e45"
SURFACE3 = "#383855"
ACCENT   = "#7c9ef5"
ACCENT_DK= "#5a7cd4"
TEXT     = "#cdd6f4"
MUTED    = "#a6adc8"
FAINT    = "#585b70"
BORDER   = "#45475a"
ENTRY_BG = "#1c1c2e"
SUCCESS  = "#a6e3a1"
ERROR    = "#f38ba8"
WARN     = "#fab387"

FONT_BODY  = ("Segoe UI", 10)
FONT_SMALL = ("Segoe UI", 9)
FONT_MONO  = ("Consolas", 9)
FONT_HEAD  = ("Segoe UI Semibold", 11)
FONT_SEMI  = ("Segoe UI Semibold", 10)

# ── reusable widgets ──────────────────────────────────────────────────────────

class PathEntry(tk.Frame):
    """Label + entry + browse button, themed."""
    def __init__(self, parent, label, default="", tip="", browse_fn=None, **kw):
        super().__init__(parent, bg=SURFACE, **kw)
        lrow = tk.Frame(self, bg=SURFACE)
        lrow.pack(fill="x")
        tk.Label(lrow, text=label, bg=SURFACE, fg=TEXT,
                 font=FONT_SEMI, anchor="w").pack(side="left")
        if tip:
            tk.Label(lrow, text=tip, bg=SURFACE, fg=FAINT,
                     font=("Segoe UI", 8), anchor="w").pack(side="left", padx=4)
        row = tk.Frame(self, bg=SURFACE)
        row.pack(fill="x", pady=(2, 0))
        self.var = tk.StringVar(value=default)
        entry = tk.Entry(row, textvariable=self.var, bg=ENTRY_BG, fg=TEXT,
                         insertbackground=TEXT, relief="flat", font=FONT_BODY,
                         highlightthickness=1, highlightbackground=BORDER,
                         highlightcolor=ACCENT)
        entry.pack(side="left", fill="x", expand=True, ipady=5, padx=(0, 6))
        if browse_fn:
            tk.Button(row, text="Browse...", command=browse_fn,
                      bg=SURFACE3, fg=TEXT, relief="flat", font=FONT_SMALL,
                      padx=10, pady=4, cursor="hand2",
                      activebackground=BORDER, activeforeground=TEXT,
                      ).pack(side="left")

    def get(self): return self.var.get().strip()
    def set(self, v): self.var.set(v)


class TextEntry(tk.Frame):
    """Label + plain entry, themed."""
    def __init__(self, parent, label, default="", tip="", **kw):
        super().__init__(parent, bg=SURFACE, **kw)
        lrow = tk.Frame(self, bg=SURFACE)
        lrow.pack(fill="x")
        tk.Label(lrow, text=label, bg=SURFACE, fg=TEXT,
                 font=FONT_SEMI, anchor="w").pack(side="left")
        if tip:
            tk.Label(lrow, text=tip, bg=SURFACE, fg=FAINT,
                     font=("Segoe UI", 8), anchor="w").pack(side="left", padx=4)
        self.var = tk.StringVar(value=default)
        entry = tk.Entry(self, textvariable=self.var, bg=ENTRY_BG, fg=TEXT,
                         insertbackground=TEXT, relief="flat", font=FONT_BODY,
                         highlightthickness=1, highlightbackground=BORDER,
                         highlightcolor=ACCENT)
        entry.pack(fill="x", ipady=5, pady=(2, 0))

    def get(self): return self.var.get().strip()
    def set(self, v): self.var.set(v)


class SliderRow(tk.Frame):
    """Label + range hint + Scale + numeric Entry, fully synced."""
    def __init__(self, parent, label, default, from_, to,
                 resolution=0.001, is_int=False, tip="", **kw):
        super().__init__(parent, bg=SURFACE, **kw)
        self.is_int   = is_int
        self.var       = tk.DoubleVar(value=float(default))
        self._updating = False

        hrow = tk.Frame(self, bg=SURFACE)
        hrow.pack(fill="x")
        tk.Label(hrow, text=label, bg=SURFACE, fg=TEXT,
                 font=FONT_SEMI, anchor="w").pack(side="left")
        tk.Label(hrow, text=f" [{from_} - {to}]", bg=SURFACE, fg=FAINT,
                 font=("Segoe UI", 8)).pack(side="left")
        if tip:
            tk.Label(hrow, text=f"  {tip}", bg=SURFACE, fg=FAINT,
                     font=("Segoe UI", 8)).pack(side="left")

        row = tk.Frame(self, bg=SURFACE)
        row.pack(fill="x", pady=(2, 0))
        self.scale = tk.Scale(
            row, from_=from_, to=to, orient="horizontal",
            variable=self.var, resolution=resolution,
            bg=SURFACE, fg=TEXT, troughcolor=ENTRY_BG,
            highlightthickness=0, sliderrelief="flat",
            activebackground=ACCENT, showvalue=False,
            command=self._on_scale)
        self.scale.pack(side="left", fill="x", expand=True)
        self.entry = tk.Entry(row, width=9, bg=ENTRY_BG, fg=ACCENT,
                              insertbackground=ACCENT, relief="flat",
                              font=FONT_MONO, justify="center",
                              highlightthickness=1, highlightbackground=BORDER,
                              highlightcolor=ACCENT)
        self.entry.insert(0, self._fmt(default))
        self.entry.pack(side="left", padx=(8, 0), ipady=5)
        self.entry.bind("<Return>",    self._on_entry)
        self.entry.bind("<FocusOut>",  self._on_entry)

    def _fmt(self, v):
        return str(int(round(v))) if self.is_int else f"{float(v):.4g}"

    def _on_scale(self, _=None):
        if self._updating:
            return
        self._updating = True
        self.entry.delete(0, "end")
        self.entry.insert(0, self._fmt(self.var.get()))
        self._updating = False

    def _on_entry(self, _=None):
        if self._updating:
            return
        try:
            v = float(self.entry.get())
            self._updating = True
            self.var.set(v)
            self._updating = False
        except ValueError:
            pass

    def get(self):
        try:
            v = float(self.entry.get())
            return int(round(v)) if self.is_int else v
        except ValueError:
            v = self.var.get()
            return int(round(v)) if self.is_int else v


class CheckRow(tk.Frame):
    """Styled checkbox."""
    def __init__(self, parent, label, default=False, tip="", **kw):
        super().__init__(parent, bg=SURFACE, **kw)
        frame = tk.Frame(self, bg=SURFACE)
        frame.pack(anchor="w", fill="x")
        self.var = tk.BooleanVar(value=default)
        cb = tk.Checkbutton(frame, text=label, variable=self.var,
                            bg=SURFACE, fg=TEXT, selectcolor=ENTRY_BG,
                            activebackground=SURFACE, activeforeground=TEXT,
                            font=FONT_SEMI, anchor="w", cursor="hand2")
        cb.pack(side="left")
        if tip:
            tk.Label(frame, text=tip, bg=SURFACE, fg=FAINT,
                     font=("Segoe UI", 8)).pack(side="left", padx=6)

    def get(self): return self.var.get()


class Section(tk.Frame):
    """Themed section with accent header bar."""
    def __init__(self, parent, title, **kw):
        super().__init__(parent, bg=SURFACE, bd=0, **kw)
        tk.Frame(self, bg=ACCENT_DK, height=2).pack(fill="x")
        tk.Label(self, text=title, bg=SURFACE2, fg=ACCENT,
                 font=FONT_HEAD, anchor="w", padx=10, pady=6,
                 ).pack(fill="x")
        self.inner = tk.Frame(self, bg=SURFACE, padx=14, pady=8)
        self.inner.pack(fill="both", expand=True)

    def pad(self):
        return dict(fill="x", pady=5)

# ── main application ──────────────────────────────────────────────────────────

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("bag-to-tileset - Docker GUI")
        self.configure(bg=BG)
        self.minsize(900, 640)
        self.geometry("1100x820")

        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Vertical.TScrollbar",
                         background=SURFACE2, troughcolor=BG,
                         arrowcolor=MUTED, bordercolor=BG, relief="flat")

        self._build_ui()
        self._update_command()

    # ── layout ────────────────────────────────────────────────────────────

    def _build_ui(self):
        # title bar
        hdr = tk.Frame(self, bg=BG, pady=10)
        hdr.pack(fill="x", padx=20)
        tk.Label(hdr, text="bag-to-tileset", bg=BG, fg=ACCENT,
                 font=("Segoe UI", 15, "bold")).pack(side="left")
        tk.Label(hdr, text=" Docker GUI", bg=BG, fg=MUTED,
                 font=("Segoe UI", 13)).pack(side="left", pady=2)
        tk.Label(hdr, text="ROS 2 Bag -> Georeferenced RGB 3D Tiles", bg=BG, fg=FAINT,
                 font=("Segoe UI", 9)).pack(side="right")

        tk.Frame(self, bg=BORDER, height=1).pack(fill="x")

        # bottom action bar
        tk.Frame(self, bg=BORDER, height=1).pack(fill="x", side="bottom")
        bar = tk.Frame(self, bg=SURFACE2, pady=8)
        bar.pack(fill="x", side="bottom")

        self.run_btn = tk.Button(
            bar, text="Run Conversion", command=self._run,
            bg=ACCENT, fg="#1e1e2e", relief="flat",
            font=("Segoe UI Semibold", 11), padx=26, pady=7, cursor="hand2",
            activebackground=ACCENT_DK, activeforeground="#1e1e2e")
        self.run_btn.pack(side="right", padx=16)

        tk.Button(bar, text="Copy Command", command=self._copy_cmd,
                  bg=SURFACE3, fg=TEXT, relief="flat", font=FONT_BODY,
                  padx=14, pady=7, cursor="hand2",
                  activebackground=BORDER, activeforeground=TEXT,
                  ).pack(side="right", padx=4)

        tk.Button(bar, text="Clear Log", command=self._clear_log,
                  bg=SURFACE3, fg=MUTED, relief="flat", font=FONT_BODY,
                  padx=12, pady=7, cursor="hand2",
                  activebackground=BORDER, activeforeground=TEXT,
                  ).pack(side="left", padx=16)

        # two-pane body
        body = tk.PanedWindow(self, orient="horizontal", bg=BG,
                              sashwidth=6, sashrelief="flat",
                              sashpad=0, opaqueresize=True)
        body.pack(fill="both", expand=True)

        # LEFT — scrollable parameter pane
        left_outer = tk.Frame(body, bg=BG)
        body.add(left_outer, minsize=420, width=520)

        canvas    = tk.Canvas(left_outer, bg=BG, highlightthickness=0, bd=0)
        scrollbar = ttk.Scrollbar(left_outer, orient="vertical",
                                   command=canvas.yview,
                                   style="Vertical.TScrollbar")
        self._param_frame = tk.Frame(canvas, bg=BG)
        self._param_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        self._cwin = canvas.create_window((0, 0), window=self._param_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfig(self._cwin, width=e.width))
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        def _scroll(e):
            delta = -1 * (e.delta // 120) if e.delta else (-1 if e.num == 4 else 1)
            canvas.yview_scroll(delta, "units")
        canvas.bind_all("<MouseWheel>", _scroll)
        canvas.bind_all("<Button-4>",   _scroll)
        canvas.bind_all("<Button-5>",   _scroll)

        self._build_params(self._param_frame)

        # RIGHT — command preview + log
        right = tk.Frame(body, bg=BG)
        body.add(right, minsize=360)
        self._build_right(right)

    # ── parameter widgets ─────────────────────────────────────────────────

    def _build_params(self, parent):
        # ── Required Inputs
        s = Section(parent, "Required Inputs")
        s.pack(fill="x", padx=8, pady=(8, 6))

        self.bag_path = PathEntry(
            s.inner, "bag_path", browse_fn=self._browse_bag,
            tip="Path to the ROS 2 .bag / .db3 / .mcap file")
        self.bag_path.pack(**s.pad())

        self.output_dir = PathEntry(
            s.inner, "output_dir", browse_fn=self._browse_outdir,
            tip="Flat directory for tileset.json + *.pnts output")
        self.output_dir.pack(**s.pad())

        self.docker_image = TextEntry(
            s.inner, "Docker Image Name", default="bag-to-tileset",
            tip="Name used when you ran 'docker build -t ...'")
        self.docker_image.pack(**s.pad())

        # ── ROS Topics
        s2 = Section(parent, "ROS Topics")
        s2.pack(fill="x", padx=8, pady=6)

        self.pc_topic = TextEntry(
            s2.inner, "--pc_topic", default="/points",
            tip="sensor_msgs/PointCloud2 topic name")
        self.pc_topic.pack(**s2.pad())

        self.odom_topic = TextEntry(
            s2.inner, "--odom_topic",
            tip="nav_msgs/Odometry topic — leave blank to omit")
        self.odom_topic.pack(**s2.pad())

        self.gps_topic = TextEntry(
            s2.inner, "--gps_topic", default="/gps/fix",
            tip="sensor_msgs/NavSatFix topic for GPS georeferencing")
        self.gps_topic.pack(**s2.pad())

        # ── Camera Colorization
        s_cam = Section(parent, "Camera Colorization (optional — leave blank to disable)")
        s_cam.pack(fill="x", padx=8, pady=6)

        self.camera_topic = TextEntry(
            s_cam.inner, "--camera_topic",
            tip="sensor_msgs/Image or CompressedImage topic — leave blank to skip colorization")
        self.camera_topic.pack(**s_cam.pad())

        self.camera_info_topic = TextEntry(
            s_cam.inner, "--camera_info_topic",
            tip="sensor_msgs/CameraInfo topic — required when camera_topic is set")
        self.camera_info_topic.pack(**s_cam.pad())

        self.max_time_diff = SliderRow(
            s_cam.inner, "--max_time_diff",
            default=0.1, from_=0.0, to=2.0, resolution=0.01,
            tip="Max camera-lidar timestamp difference (s)")
        self.max_time_diff.pack(**s_cam.pad())

        self.color_min_depth = SliderRow(
            s_cam.inner, "--color_min_depth",
            default=0.1, from_=0.0, to=5.0, resolution=0.05,
            tip="Min Euclidean depth for projection (m) — points closer get gray fill")
        self.color_min_depth.pack(**s_cam.pad())

        self.color_max_depth = TextEntry(
            s_cam.inner, "--color_max_depth", default="",
            tip="Max Euclidean depth for projection (m) — leave blank for no limit")
        self.color_max_depth.pack(**s_cam.pad())

        self.gray_filter_radius = SliderRow(
            s_cam.inner, "--gray_filter_radius",
            default=0.05, from_=0.0, to=1.0, resolution=0.01,
            tip="Remove gray fill points within this distance of coloured points (m)")
        self.gray_filter_radius.pack(**s_cam.pad())

        # ── Registration (ICP)
        s3 = Section(parent, "Registration (ICP)")
        s3.pack(fill="x", padx=8, pady=6)

        self.voxel_size = SliderRow(
            s3.inner, "--voxel_size",
            default=0.05, from_=0.001, to=1.0, resolution=0.001,
            tip="Downsampling resolution (m)")
        self.voxel_size.pack(**s3.pad())

        self.icp_dist = SliderRow(
            s3.inner, "--icp_dist_thresh",
            default=0.2, from_=0.01, to=10.0, resolution=0.01,
            tip="Max point correspondence distance (m)")
        self.icp_dist.pack(**s3.pad())

        self.icp_fitness = SliderRow(
            s3.inner, "--icp_fitness_thresh",
            default=0.6, from_=0.0, to=1.0, resolution=0.01,
            tip="Min fraction of points aligned to accept a frame [0-1]")
        self.icp_fitness.pack(**s3.pad())

        self.odom_latency = SliderRow(
            s3.inner, "--odom_max_latency",
            default=0.5, from_=0.0, to=5.0, resolution=0.05,
            tip="Max odom <-> pointcloud timestamp gap (s)")
        self.odom_latency.pack(**s3.pad())

        # ── Loop Closure
        s_lc = Section(parent, "Loop Closure (optional)")
        s_lc.pack(fill="x", padx=8, pady=6)

        self.enable_loop_closure = CheckRow(
            s_lc.inner, "--enable_loop_closure",
            tip="Enable RANSAC+ICP loop closure detection")
        self.enable_loop_closure.pack(**s_lc.pad())

        self.lc_radius = SliderRow(
            s_lc.inner, "--loop_closure_radius",
            default=5.0, from_=0.5, to=50.0, resolution=0.5,
            tip="Spatial search radius for loop closure candidates (m)")
        self.lc_radius.pack(**s_lc.pad())

        self.lc_fitness = SliderRow(
            s_lc.inner, "--loop_closure_fitness_thresh",
            default=0.3, from_=0.0, to=1.0, resolution=0.01,
            tip="Min ICP fitness to accept a loop closure edge")
        self.lc_fitness.pack(**s_lc.pad())

        self.lc_interval = SliderRow(
            s_lc.inner, "--loop_closure_search_interval",
            default=10, from_=1, to=100, resolution=1, is_int=True,
            tip="Check for loop closures every N frames")
        self.lc_interval.pack(**s_lc.pad())

        # ── Cleaning
        s4 = Section(parent, "Outlier Removal & Cleaning")
        s4.pack(fill="x", padx=8, pady=6)

        self.ror_nb = SliderRow(
            s4.inner, "--ror_nb_points",
            default=6, from_=1, to=30, resolution=1, is_int=True,
            tip="ROR: min neighbours within radius")
        self.ror_nb.pack(**s4.pad())

        self.ror_radius = SliderRow(
            s4.inner, "--ror_radius",
            default=0.5, from_=0.05, to=5.0, resolution=0.05,
            tip="ROR: search radius (m)")
        self.ror_radius.pack(**s4.pad())

        self.sor_nb = SliderRow(
            s4.inner, "--sor_nb_neighbors",
            default=20, from_=1, to=100, resolution=1, is_int=True,
            tip="SOR: neighbour count for mean-distance estimate")
        self.sor_nb.pack(**s4.pad())

        self.sor_std = SliderRow(
            s4.inner, "--sor_std_ratio",
            default=2.0, from_=0.5, to=10.0, resolution=0.1,
            tip="SOR: std-deviation multiplier for outlier threshold")
        self.sor_std.pack(**s4.pad())

        self.dbscan_eps = SliderRow(
            s4.inner, "--dbscan_eps",
            default=0.5, from_=0.0, to=5.0, resolution=0.05,
            tip="DBSCAN epsilon (m) — set 0 to disable")
        self.dbscan_eps.pack(**s4.pad())

        self.dbscan_min = SliderRow(
            s4.inner, "--dbscan_min_points",
            default=10, from_=1, to=100, resolution=1, is_int=True,
            tip="DBSCAN minimum cluster size")
        self.dbscan_min.pack(**s4.pad())

        # ── Output Options
        s5 = Section(parent, "Output Options")
        s5.pack(fill="x", padx=8, pady=(6, 12))

        self.level_floor = CheckRow(
            s5.inner, "--level_floor",
            tip="Attempt RANSAC floor leveling before export")
        self.level_floor.pack(**s5.pad())

        self.workers = SliderRow(
            s5.inner, "--workers",
            default=4, from_=1, to=32, resolution=1, is_int=True,
            tip="py3dtiles worker threads")
        self.workers.pack(**s5.pad())

        # bind live command preview
        for widget in self._all_vars():
            widget.trace_add("write", self._update_command)

    def _all_vars(self):
        """Yield all tk.Variable objects owned by parameter widgets."""
        for attr in vars(self).values():
            if isinstance(attr, (PathEntry, TextEntry, SliderRow, CheckRow)):
                yield attr.var

    # ── right pane ────────────────────────────────────────────────────────

    def _build_right(self, parent):
        # Command preview
        cmd_lbl = tk.Frame(parent, bg=BG, pady=6)
        cmd_lbl.pack(fill="x", padx=12)
        tk.Label(cmd_lbl, text="Docker Command Preview", bg=BG, fg=MUTED,
                 font=FONT_SEMI).pack(side="left")

        self.cmd_text = tk.Text(
            parent, height=8, bg=SURFACE, fg=ACCENT,
            insertbackground=ACCENT, relief="flat", font=FONT_MONO,
            padx=10, pady=8, wrap="none",
            highlightthickness=1, highlightbackground=BORDER)
        self.cmd_text.pack(fill="x", padx=12, pady=(0, 4))
        self.cmd_text.configure(state="disabled")

        tk.Frame(parent, bg=BORDER, height=1).pack(fill="x", padx=12, pady=4)

        # Log
        log_lbl = tk.Frame(parent, bg=BG, pady=4)
        log_lbl.pack(fill="x", padx=12)
        tk.Label(log_lbl, text="Output Log", bg=BG, fg=MUTED,
                 font=FONT_SEMI).pack(side="left")

        self.log = scrolledtext.ScrolledText(
            parent, bg=SURFACE, fg=TEXT,
            insertbackground=TEXT, relief="flat", font=FONT_MONO,
            padx=10, pady=8, wrap="word",
            highlightthickness=1, highlightbackground=BORDER)
        self.log.pack(fill="both", expand=True, padx=12, pady=(0, 4))
        self.log.configure(state="disabled")

        # log colour tags
        self.log.tag_configure("ok",   foreground=SUCCESS)
        self.log.tag_configure("err",  foreground=ERROR)
        self.log.tag_configure("warn", foreground=WARN)
        self.log.tag_configure("info", foreground=ACCENT)
        self.log.tag_configure("cmd",  foreground=MUTED)
        self.log.tag_configure("muted",foreground=FAINT)

    # ── browse helpers ────────────────────────────────────────────────────

    def _browse_bag(self):
        path = filedialog.askopenfilename(
            title="Select ROS 2 bag file",
            filetypes=[("Bag files", "*.bag *.db3 *.mcap"), ("All files", "*.*")])
        if path:
            self.bag_path.set(path)

    def _browse_outdir(self):
        path = filedialog.askdirectory(title="Select output directory")
        if path:
            self.output_dir.set(path)

    # ── command builder ───────────────────────────────────────────────────

    def _build_parts(self):
        """Return (list_of_cmd_parts, None) or (None, error_str)."""
        bag   = self.bag_path.get()
        outd  = self.output_dir.get()
        image = self.docker_image.get() or "bag-to-tileset"

        if not bag or not outd:
            return None, "bag_path and output_dir are required."

        bag_p  = Path(bag)
        out_p  = Path(outd)
        bag_dir  = str(bag_p.parent.resolve())
        bag_file = bag_p.name
        out_host = str(out_p.resolve())

        parts = [
            "docker", "run", "--rm",
            "-v", f"{bag_dir}:/bag",
            "-v", f"{out_host}:/output",
            image,
            f"/bag/{bag_file}", "/output",
        ]

        # topics
        parts += ["--pc_topic",  self.pc_topic.get() or "/points"]
        parts += ["--gps_topic", self.gps_topic.get() or "/gps/fix"]
        odom = self.odom_topic.get()
        if odom:
            parts += ["--odom_topic", odom]

        # camera colorization
        cam = self.camera_topic.get()
        if cam:
            parts += ["--camera_topic", cam]
            cam_info = self.camera_info_topic.get()
            if cam_info:
                parts += ["--camera_info_topic", cam_info]
            parts += ["--max_time_diff",      str(self.max_time_diff.get())]
            parts += ["--color_min_depth",    str(self.color_min_depth.get())]
            cmd = self.color_max_depth.get()
            if cmd:
                parts += ["--color_max_depth", cmd]
            parts += ["--gray_filter_radius", str(self.gray_filter_radius.get())]

        # ICP
        parts += ["--voxel_size",         str(self.voxel_size.get())]
        parts += ["--icp_dist_thresh",    str(self.icp_dist.get())]
        parts += ["--icp_fitness_thresh", str(self.icp_fitness.get())]
        parts += ["--odom_max_latency",   str(self.odom_latency.get())]

        # loop closure
        if self.enable_loop_closure.get():
            parts += ["--enable_loop_closure"]
            parts += ["--loop_closure_radius",           str(self.lc_radius.get())]
            parts += ["--loop_closure_fitness_thresh",   str(self.lc_fitness.get())]
            parts += ["--loop_closure_search_interval",  str(self.lc_interval.get())]

        # cleaning
        parts += ["--ror_nb_points",     str(self.ror_nb.get())]
        parts += ["--ror_radius",        str(self.ror_radius.get())]
        parts += ["--sor_nb_neighbors",  str(self.sor_nb.get())]
        parts += ["--sor_std_ratio",     str(self.sor_std.get())]
        parts += ["--dbscan_eps",        str(self.dbscan_eps.get())]
        parts += ["--dbscan_min_points", str(self.dbscan_min.get())]

        # misc
        if self.level_floor.get():
            parts += ["--level_floor"]
        wk = self.workers.get()
        if wk != 4:
            parts += ["--workers", str(wk)]

        return parts, None

    @staticmethod
    def _format_cmd(parts):
        """Pretty-print the docker command with backslash line continuations."""
        if not parts:
            return ""
        lines = []
        i = 0
        while i < len(parts):
            tok = parts[i]
            if tok in ("-v", "-e", "-p", "-u") and i + 1 < len(parts):
                lines.append(f"  {tok} {shlex.quote(parts[i + 1])}")
                i += 2
            elif (tok.startswith("--") and i + 1 < len(parts)
                  and not parts[i + 1].startswith("-")):
                lines.append(f"  {tok} {shlex.quote(parts[i + 1])}")
                i += 2
            else:
                lines.append(tok if i == 0 else f"  {tok}")
                i += 1
        return " \\\n".join(lines)

    def _update_command(self, *_):
        parts, err = self._build_parts()
        text = self._format_cmd(parts) if parts else f"# {err}"
        self.cmd_text.configure(state="normal")
        self.cmd_text.delete("1.0", "end")
        self.cmd_text.insert("end", text)
        self.cmd_text.configure(state="disabled")

    def _copy_cmd(self):
        parts, err = self._build_parts()
        if parts:
            self.clipboard_clear()
            self.clipboard_append(self._format_cmd(parts))
            self._log("Command copied to clipboard.", "muted")
        else:
            messagebox.showwarning("Incomplete", err)

    # ── logging ───────────────────────────────────────────────────────────

    def _log(self, text, tag=""):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    # ── run ───────────────────────────────────────────────────────────────

    def _run(self):
        parts, err = self._build_parts()
        if parts is None:
            messagebox.showerror("Missing Input",
                                  "Please specify both bag_path and output_dir.")
            return

        out_p = Path(self.output_dir.get())
        try:
            out_p.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            messagebox.showerror("Error", f"Could not create output directory:\n{e}")
            return

        cmd_str = self._format_cmd(parts)
        self._log(f"\n{'--' * 28}", "muted")
        self._log("$ " + cmd_str, "cmd")
        self._log(f"{'--' * 28}\n", "muted")

        self.run_btn.configure(state="disabled", text="Running...",
                                bg=SURFACE3, fg=MUTED)

        def _worker():
            try:
                proc = subprocess.Popen(
                    parts,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True, bufsize=1)
                for raw_line in proc.stdout:
                    line = raw_line.rstrip()
                    lo   = line.lower()
                    if any(k in lo for k in ("error", "traceback", "exception")):
                        tag = "err"
                    elif any(k in lo for k in ("warning", "warn")):
                        tag = "warn"
                    elif any(k in lo for k in ("saved", "done", "complete",
                                                "success", "tiles written",
                                                "tileset", "colours: yes")):
                        tag = "ok"
                    elif any(k in lo for k in ("registering", "reading",
                                                "optimizing", "merging",
                                                "georeferencing", "generating",
                                                "extracted", "gps", "coloring",
                                                "gray filter")):
                        tag = "info"
                    else:
                        tag = ""
                    self.after(0, self._log, line, tag)
                proc.wait()
                if proc.returncode == 0:
                    self.after(0, self._log, "\nConversion complete!", "ok")
                    self.after(0, self._log,
                               f"  Tileset: {self.output_dir.get()}/tileset.json", "ok")
                else:
                    self.after(0, self._log,
                               f"\nProcess exited with code {proc.returncode}", "err")
            except FileNotFoundError:
                self.after(0, self._log,
                           "docker not found — is Docker installed and on PATH?", "err")
            except Exception as exc:
                self.after(0, self._log, f"Unexpected error: {exc}", "err")
            finally:
                self.after(0, lambda: self.run_btn.configure(
                    state="normal", text="Run Conversion",
                    bg=ACCENT, fg="#1e1e2e"))

        threading.Thread(target=_worker, daemon=True).start()

# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = App()
    app.mainloop()
