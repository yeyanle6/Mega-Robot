import os

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAction,
    QActionGroup,
    QDialog,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QSpinBox,
    QToolBar,
)

from map_repair_qt.canvas import (
    TOOL_BRUSH,
    TOOL_ERASER,
    TOOL_LINE,
    TOOL_RECT,
    MapCanvas,
)
from map_repair_qt.document import MapDocument

TOOL_LABELS = {
    TOOL_BRUSH: "\u753b\u7b14",
    TOOL_RECT: "\u77e9\u5f62",
    TOOL_LINE: "\u76f4\u7ebf",
    TOOL_ERASER: "\u6a61\u76ae\u64e6",
}


class MapRepairMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.document = MapDocument()
        self.document.changed.connect(self._refresh_title)
        self.document.metadata_changed.connect(self._refresh_title)

        self.canvas = MapCanvas(self.document)
        self.canvas.hover_changed.connect(self._on_hover_changed)
        self.canvas.tool_changed.connect(self._on_tool_changed)
        self.setCentralWidget(self.canvas)

        self.hover_label = QLabel("\u5355\u5143\u683c: -, -  \u503c: -")
        self.zoom_label = QLabel("\u7f29\u653e: 1200%")
        self.meta_label = QLabel("\u5206\u8fa8\u7387: -")
        self.tool_label = QLabel("\u5de5\u5177: \u753b\u7b14")
        self.statusBar().addWidget(self.hover_label)
        self.statusBar().addPermanentWidget(self.tool_label)
        self.statusBar().addPermanentWidget(self.meta_label)
        self.statusBar().addPermanentWidget(self.zoom_label)

        self._create_actions()
        self._create_toolbar()
        self._create_menus()

        self.resize(1400, 900)
        self.setWindowTitle("\u5730\u56fe\u4fee\u590d\u5de5\u5177")
        self._refresh_title()

    def _create_actions(self):
        # File
        self.open_action = QAction("\u6253\u5f00\u5730\u56fe", self)
        self.open_action.setShortcut("Ctrl+O")
        self.open_action.triggered.connect(self.open_map_dialog)

        self.save_action = QAction("\u4fdd\u5b58", self)
        self.save_action.setShortcut("Ctrl+S")
        self.save_action.triggered.connect(self.save_map)

        self.save_as_action = QAction("\u53e6\u5b58\u4e3a", self)
        self.save_as_action.setShortcut("Ctrl+Shift+S")
        self.save_as_action.triggered.connect(self.save_map_as)

        # Edit
        self.undo_action = QAction("\u64a4\u9500", self)
        self.undo_action.setShortcut("Ctrl+Z")
        self.undo_action.triggered.connect(self.document.undo)

        self.redo_action = QAction("\u91cd\u505a", self)
        self.redo_action.setShortcut("Ctrl+Y")
        self.redo_action.triggered.connect(self.document.redo)

        # View
        self.fit_action = QAction("\u9002\u914d\u7a97\u53e3", self)
        self.fit_action.setShortcut("F")
        self.fit_action.triggered.connect(self.canvas.fit_to_window)

        self.reset_view_action = QAction("\u91cd\u7f6e\u89c6\u56fe", self)
        self.reset_view_action.triggered.connect(self.canvas.reset_view)

        self.grid_action = QAction("\u7f51\u683c", self)
        self.grid_action.setCheckable(True)
        self.grid_action.setChecked(True)
        self.grid_action.setShortcut("G")
        self.grid_action.toggled.connect(self.canvas.set_show_grid)

        # Tool group
        self.tool_group = QActionGroup(self)
        self.tool_group.setExclusive(True)
        self.tool_actions = {}

        tool_specs = [
            (TOOL_BRUSH, "\u753b\u7b14", "B"),
            (TOOL_RECT, "\u77e9\u5f62", "R"),
            (TOOL_LINE, "\u76f4\u7ebf", "L"),
            (TOOL_ERASER, "\u6a61\u76ae\u64e6", "E"),
        ]
        for tool_id, label, shortcut in tool_specs:
            action = QAction(label, self)
            action.setCheckable(True)
            action.setShortcut(shortcut)
            action.triggered.connect(
                lambda checked, t=tool_id: self.canvas.set_tool(t))
            self.tool_group.addAction(action)
            self.tool_actions[tool_id] = action

        self.tool_actions[TOOL_BRUSH].setChecked(True)

        # Color group
        self.brush_group = QActionGroup(self)
        self.brush_group.setExclusive(True)
        self.brush_actions = {}

        brush_specs = [
            ("occupied", "\u969c\u788d\u7269", "1"),
            ("free", "\u53ef\u901a\u884c", "2"),
            ("unknown", "\u672a\u77e5", "3"),
        ]
        for mode, label, shortcut in brush_specs:
            action = QAction(label, self)
            action.setCheckable(True)
            action.setShortcut(shortcut)
            action.triggered.connect(
                lambda checked, m=mode: self.canvas.set_brush_mode(m))
            self.brush_group.addAction(action)
            self.brush_actions[mode] = action

        self.brush_actions["occupied"].setChecked(True)

        # Denoise
        self.denoise_action = QAction("\u53bb\u566a\u2026", self)
        self.denoise_action.triggered.connect(self._run_denoise)

    def _create_toolbar(self):
        toolbar = QToolBar("\u5de5\u5177\u680f", self)
        toolbar.setMovable(False)
        self.addToolBar(Qt.TopToolBarArea, toolbar)

        toolbar.addAction(self.open_action)
        toolbar.addAction(self.save_action)
        toolbar.addSeparator()
        toolbar.addAction(self.undo_action)
        toolbar.addAction(self.redo_action)
        toolbar.addSeparator()

        for action in self.tool_actions.values():
            toolbar.addAction(action)

        toolbar.addSeparator()

        for action in self.brush_actions.values():
            toolbar.addAction(action)

        toolbar.addSeparator()
        toolbar.addAction(self.grid_action)
        toolbar.addAction(self.fit_action)
        toolbar.addAction(self.reset_view_action)
        toolbar.addSeparator()

        toolbar.addWidget(QLabel(" \u5927\u5c0f "))
        self.brush_size_spin = QSpinBox(self)
        self.brush_size_spin.setRange(1, 16)
        self.brush_size_spin.setValue(1)
        self.brush_size_spin.valueChanged.connect(self.canvas.set_brush_size)
        toolbar.addWidget(self.brush_size_spin)

    def _create_menus(self):
        file_menu = self.menuBar().addMenu("\u6587\u4ef6")
        file_menu.addAction(self.open_action)
        file_menu.addAction(self.save_action)
        file_menu.addAction(self.save_as_action)

        edit_menu = self.menuBar().addMenu("\u7f16\u8f91")
        edit_menu.addAction(self.undo_action)
        edit_menu.addAction(self.redo_action)

        view_menu = self.menuBar().addMenu("\u89c6\u56fe")
        view_menu.addAction(self.grid_action)
        view_menu.addAction(self.fit_action)
        view_menu.addAction(self.reset_view_action)

        tools_menu = self.menuBar().addMenu("\u5de5\u5177")
        for action in self.tool_actions.values():
            tools_menu.addAction(action)
        tools_menu.addSeparator()
        tools_menu.addAction(self.denoise_action)

    def open_map_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "\u6253\u5f00 ROS \u5730\u56fe YAML", "",
            "YAML \u6587\u4ef6 (*.yaml *.yml)",
        )
        if path:
            self.load_map(path)

    def load_map(self, yaml_path):
        try:
            self.document.load_yaml(yaml_path)
            self.canvas.fit_to_window()
            self._update_meta_label()
        except Exception as exc:
            QMessageBox.critical(self, "\u6253\u5f00\u5730\u56fe\u5931\u8d25",
                                 str(exc))

    def save_map(self):
        if not self.document.has_map():
            return
        if self.document.yaml_path:
            try:
                self.document.save_yaml(self.document.yaml_path,
                                        self.document.image_path)
                self._refresh_title()
            except Exception as exc:
                QMessageBox.critical(self, "\u4fdd\u5b58\u5931\u8d25",
                                     str(exc))
            return
        self.save_map_as()

    def save_map_as(self):
        if not self.document.has_map():
            return

        yaml_path, _ = QFileDialog.getSaveFileName(
            self, "\u4fdd\u5b58 ROS \u5730\u56fe YAML",
            self.document.yaml_path or "",
            "YAML \u6587\u4ef6 (*.yaml)",
        )
        if not yaml_path:
            return

        default_image = os.path.splitext(yaml_path)[0] + ".pgm"
        image_path, _ = QFileDialog.getSaveFileName(
            self, "\u4fdd\u5b58\u5730\u56fe\u56fe\u50cf", default_image,
            "PGM \u6587\u4ef6 (*.pgm);;PNG \u6587\u4ef6 (*.png)",
        )
        if not image_path:
            return

        try:
            rel_image = os.path.relpath(image_path, os.path.dirname(yaml_path))
            self.document.save_yaml(yaml_path, rel_image)
            self._refresh_title()
        except Exception as exc:
            QMessageBox.critical(self, "\u4fdd\u5b58\u5931\u8d25", str(exc))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.zoom_label.setText(
            f"\u7f29\u653e: {int(self.canvas.zoom * 100)}%")

    def _on_hover_changed(self, x, y, value):
        self.hover_label.setText(
            f"\u5355\u5143\u683c: {x}, {y}  \u503c: {value}")
        self.zoom_label.setText(
            f"\u7f29\u653e: {int(self.canvas.zoom * 100)}%")

    def _on_tool_changed(self, tool):
        self.tool_label.setText(
            f"\u5de5\u5177: {TOOL_LABELS.get(tool, tool)}")

    def _update_meta_label(self):
        if not self.document.has_map():
            self.meta_label.setText("\u5206\u8fa8\u7387: -")
            return
        self.meta_label.setText(
            f"\u5206\u8fa8\u7387: {self.document.metadata.resolution:.3f} m/cell"
            f"  \u5c3a\u5bf8: {self.document.width}\u00d7{self.document.height}"
        )

    def _run_denoise(self):
        if not self.document.has_map():
            return

        from map_repair_qt.denoise import HAS_CV2, DenoiseDialog, denoise

        if not HAS_CV2:
            QMessageBox.critical(
                self, "\u7f3a\u5c11\u4f9d\u8d56",
                "\u53bb\u566a\u529f\u80fd\u9700\u8981 python3-opencv\u3002\n"
                "\u8bf7\u6267\u884c: sudo apt install python3-opencv")
            return

        dialog = DenoiseDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return

        changes, num_components, num_pixels = denoise(
            self.document, dialog.mode, dialog.min_size)

        if changes:
            self.document.apply_stroke(changes)
            verb = ("\u5220\u9664" if dialog.mode == "remove_obstacles"
                    else "\u586b\u8865")
            QMessageBox.information(
                self, "\u53bb\u566a\u5b8c\u6210",
                f"\u5df2{verb} {num_components} \u4e2a\u8fde\u901a\u57df"
                f"\uff08\u5171 {num_pixels} \u50cf\u7d20\uff09")
        else:
            QMessageBox.information(
                self, "\u53bb\u566a\u5b8c\u6210",
                "\u672a\u627e\u5230\u7b26\u5408\u6761\u4ef6\u7684\u8fde\u901a\u57df\u3002")

    def _refresh_title(self):
        name = self.document.yaml_path or "\u672a\u547d\u540d"
        dirty = " *" if self.document.dirty else ""
        self.setWindowTitle(
            f"\u5730\u56fe\u4fee\u590d\u5de5\u5177 - {name}{dirty}")
        self._update_meta_label()
