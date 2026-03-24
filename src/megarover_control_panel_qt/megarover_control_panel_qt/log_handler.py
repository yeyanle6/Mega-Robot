#!/usr/bin/env python3
"""
Log handler module for reading and processing subprocess output.
"""

from PyQt5.QtCore import QThread, pyqtSignal
import re


class LogReaderThread(QThread):
    """Thread to read subprocess output asynchronously."""

    log_received = pyqtSignal(str, str)  # (level, message)

    def __init__(self, process, component_id):
        super().__init__()
        self.process = process
        self.component_id = component_id
        self.running = True

    def run(self):
        """Main thread loop to read process output."""
        try:
            for line in iter(self.process.stdout.readline, ''):
                if not self.running:
                    break

                if line:
                    line = line.rstrip('\n')
                    level = self._detect_log_level(line)
                    self.log_received.emit(level, line)
        except Exception as e:
            self.log_received.emit('ERROR', f'LogReader exception: {str(e)}')

    def _detect_log_level(self, message):
        """Detect log level from message content."""
        message_upper = message.upper()

        if any(keyword in message_upper for keyword in ['ERROR', 'FATAL', 'EXCEPTION', 'FAILED']):
            return 'ERROR'
        elif any(keyword in message_upper for keyword in ['WARN', 'WARNING']):
            return 'WARN'
        elif any(keyword in message_upper for keyword in ['INFO', 'STARTED', 'RUNNING', 'SUCCESS']):
            return 'INFO'
        elif 'DEBUG' in message_upper:
            return 'DEBUG'
        else:
            return 'INFO'

    def stop(self):
        """Stop the thread."""
        self.running = False


class LogFormatter:
    """Formatter for log messages with color support."""

    # ANSI color codes for terminal
    COLORS = {
        'ERROR': '\033[91m',   # Red
        'WARN': '\033[93m',    # Yellow
        'INFO': '\033[92m',    # Green
        'DEBUG': '\033[94m',   # Blue
        'RESET': '\033[0m'
    }

    # HTML colors for Qt text widgets
    HTML_COLORS = {
        'ERROR': '#FF5555',
        'WARN': '#FFB86C',
        'INFO': '#50FA7B',
        'DEBUG': '#8BE9FD'
    }

    @staticmethod
    def format_html(level, message, component_id):
        """Format log message as HTML with color."""
        color = LogFormatter.HTML_COLORS.get(level, '#FFFFFF')
        timestamp = LogFormatter._get_timestamp()

        # Escape HTML characters
        message = message.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')

        return f'<span style="color: #888888;">[{timestamp}]</span> ' \
               f'<span style="color: #50C878;">[{component_id}]</span> ' \
               f'<span style="color: {color};">{message}</span>'

    @staticmethod
    def _get_timestamp():
        """Get current timestamp string."""
        from datetime import datetime
        return datetime.now().strftime('%H:%M:%S')

    @staticmethod
    def strip_ansi(text):
        """Remove ANSI escape codes from text."""
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        return ansi_escape.sub('', text)
