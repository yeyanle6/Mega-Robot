#!/usr/bin/env python3
"""
Map Save Progress Dialog
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QProgressBar, QTextEdit, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont


class MapSaveProgressDialog(QDialog):
    """地图保存进度对话框"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('保存地图')
        self.setModal(True)
        self.setMinimumWidth(500)
        self.setMinimumHeight(300)

        self.init_ui()

    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()

        # 标题
        title_label = QLabel('🗺️ 正在保存地图...')
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # 状态消息
        self.status_label = QLabel('准备开始...')
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        layout.addWidget(self.progress_bar)

        # 详细信息组
        details_group = QGroupBox("保存详情")
        details_layout = QVBoxLayout()

        self.details_text = QTextEdit()
        self.details_text.setReadOnly(True)
        self.details_text.setMaximumHeight(150)
        self.details_text.setStyleSheet("""
            QTextEdit {
                background-color: #2b2b2b;
                color: #e0e0e0;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 10pt;
                border: 1px solid #555;
                border-radius: 3px;
            }
        """)
        details_layout.addWidget(self.details_text)
        details_group.setLayout(details_layout)
        layout.addWidget(details_group)

        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        self.cancel_btn = QPushButton('取消')
        self.cancel_btn.setMinimumWidth(100)
        self.cancel_btn.clicked.connect(self.on_cancel_clicked)
        button_layout.addWidget(self.cancel_btn)

        self.close_btn = QPushButton('关闭')
        self.close_btn.setMinimumWidth(100)
        self.close_btn.setEnabled(False)
        self.close_btn.clicked.connect(self.accept)
        button_layout.addWidget(self.close_btn)

        layout.addLayout(button_layout)

        self.setLayout(layout)

        # 初始化计时器（用于动画效果）
        self.dots_count = 0
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.update_animation)
        self.animation_timer.start(500)  # 每 0.5 秒更新一次

    def update_progress(self, progress, message):
        """更新进度"""
        self.progress_bar.setValue(progress)
        self.status_label.setText(message)

        # 添加到详情
        timestamp = QTimer().currentTime().toString('hh:mm:ss')
        self.details_text.append(f'[{timestamp}] {message}')

        # 自动滚动到底部
        self.details_text.verticalScrollBar().setValue(
            self.details_text.verticalScrollBar().maximum()
        )

    def update_animation(self):
        """更新动画效果"""
        if self.progress_bar.value() < 100:
            self.dots_count = (self.dots_count + 1) % 4
            dots = '.' * self.dots_count
            current_text = self.status_label.text().rstrip('.')
            self.status_label.setText(f'{current_text}{dots}')

    def on_cancel_clicked(self):
        """取消按钮点击"""
        self.reject()

    def set_completed(self):
        """设置为完成状态"""
        self.animation_timer.stop()
        self.cancel_btn.setEnabled(False)
        self.close_btn.setEnabled(True)
        self.progress_bar.setValue(100)

    def set_error(self):
        """设置为错误状态"""
        self.animation_timer.stop()
        self.cancel_btn.setEnabled(False)
        self.close_btn.setEnabled(True)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #E74C3C;
            }
        """)


class MapSaveResultDialog(QDialog):
    """地图保存结果对话框"""

    def __init__(self, success, result, parent=None):
        super().__init__(parent)
        self.success = success
        self.result = result
        self.setWindowTitle('保存结果')
        self.setModal(True)
        self.setMinimumWidth(500)

        self.init_ui()

    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()

        # 标题
        if self.success:
            title_text = '✅ 地图保存成功！'
            title_color = '#27AE60'
        else:
            title_text = '⚠️ 地图部分保存'
            title_color = '#F39C12'

        title_label = QLabel(title_text)
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet(f'color: {title_color};')
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # 分隔线
        line = QLabel()
        line.setFrameStyle(QLabel.HLine | QLabel.Sunken)
        layout.addWidget(line)

        # 保存状态
        status_layout = QHBoxLayout()

        # 2D 地图状态
        map_2d_status = '✓' if self.result.get('map_2d_saved') else '✗'
        map_2d_color = '#27AE60' if self.result.get('map_2d_saved') else '#E74C3C'
        map_2d_label = QLabel(f'<span style="color: {map_2d_color}; font-size: 18pt;">{map_2d_status}</span> 2D 栅格地图')
        status_layout.addWidget(map_2d_label)

        status_layout.addStretch()

        # 3D 地图状态
        map_3d_status = '✓' if self.result.get('map_3d_saved') else '✗'
        map_3d_color = '#27AE60' if self.result.get('map_3d_saved') else '#E74C3C'
        map_3d_label = QLabel(f'<span style="color: {map_3d_color}; font-size: 18pt;">{map_3d_status}</span> 3D 点云地图')
        status_layout.addWidget(map_3d_label)

        layout.addLayout(status_layout)

        # 文件列表
        if self.result.get('files'):
            files_group = QGroupBox("已保存的文件")
            files_layout = QVBoxLayout()

            for file_info in self.result['files']:
                size_mb = file_info['size'] / (1024 * 1024)
                if size_mb < 1:
                    size_str = f"{file_info['size'] / 1024:.1f} KB"
                else:
                    size_str = f"{size_mb:.1f} MB"

                file_label = QLabel(
                    f"• <b>{file_info['name']}</b> - {size_str} ({file_info['type']})"
                )
                files_layout.addWidget(file_label)

            files_group.setLayout(files_layout)
            layout.addWidget(files_group)

        # 使用说明
        if self.result.get('map_2d_saved') and self.result.get('map_3d_saved'):
            usage_group = QGroupBox("使用方法")
            usage_layout = QVBoxLayout()

            usage_text = QLabel(
                '在控制面板中启动 <b>"导航 (PGO地图)"</b> 模式时：<br>'
                '1. 第一次选择 <b>.yaml</b> 文件（2D 地图）<br>'
                '2. 第二次选择 <b>.pcd</b> 文件（3D 点云）'
            )
            usage_text.setWordWrap(True)
            usage_layout.addWidget(usage_text)

            usage_group.setLayout(usage_layout)
            layout.addWidget(usage_group)
        elif not self.result.get('map_3d_saved'):
            # 3D 地图未保存的提示
            warning_label = QLabel(
                '⚠️ <b>3D 点云地图未保存</b><br><br>'
                '可能原因：<br>'
                '• FastLIO 数据库为空（~/.ros/fastlio_maps/）<br>'
                '• SLAM 未运行足够长时间建立地图<br><br>'
                '建议：<br>'
                '• 继续建图后重新保存<br>'
                '• 检查 ~/.ros/fastlio_maps/ 目录是否有 .pcd 文件'
            )
            warning_label.setWordWrap(True)
            warning_label.setStyleSheet('color: #F39C12; padding: 10px; background-color: #FFF3CD; border-radius: 5px;')
            layout.addWidget(warning_label)

        # 关闭按钮
        close_btn = QPushButton('确定')
        close_btn.setMinimumWidth(100)
        close_btn.clicked.connect(self.accept)
        close_btn_layout = QHBoxLayout()
        close_btn_layout.addStretch()
        close_btn_layout.addWidget(close_btn)
        close_btn_layout.addStretch()
        layout.addLayout(close_btn_layout)

        self.setLayout(layout)
