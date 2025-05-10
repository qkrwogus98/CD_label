import pandas as pd
import cv2
import numpy as np
import os
import sys
import traceback
from PyQt5.QtCore import (
    Qt, QPoint, QEvent, QTimer, QStringListModel, QSize
)
from PyQt5.QtGui import (
    QPainter, QPen, QColor, QBrush, QPolygon, QPixmap, QIntValidator, QCursor, QMouseEvent, QFont, QImage
)
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QAction,
    QMessageBox,
    QPushButton,
    QLabel,
    QLineEdit,
    QHBoxLayout,
    QVBoxLayout,
    QListView,
    QInputDialog,
    QFileDialog,
    QMenu,
    QFontDialog,
    QSplitter,
    QGridLayout,
    QScrollArea,
    QSlider,
)
from PyQt5.QtGui import QStandardItemModel, QStandardItem
import webbrowser
from PyQt5.QtWidgets import QInputDialog
from PyQt5.QtCore import QTimer
from collections import defaultdict
import re


def natural_key(filename):
    """숫자가 포함된 문자열을 숫자 기준으로 정렬"""
    return [
        int(text) if text.isdigit() else text.lower()
        for text in re.split("([0-9]+)", filename)
    ]


def save_label_statistics(image_labels, output_dir):
    label_to_files = defaultdict(set)

    for image_path, poly_list in image_labels.items():
        used_classes = set(poly["class"] for poly in poly_list)
        for cls in used_classes:
            class_name = {1: "사람", 2: "돌", 3: "흙", 4: "물", 5: "불", 6: "나무"}.get(
                cls
            )
            if class_name:
                label_to_files[class_name].add(os.path.basename(image_path))

    os.makedirs(output_dir, exist_ok=True)
    for label, files in label_to_files.items():
        with open(os.path.join(output_dir, f"{label}.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(sorted(files)))


def append_no_change_file(output_dir, selected_a_image_path, selected_b_image_path):
    os.makedirs(output_dir, exist_ok=True)
    no_change_file_list = [
        os.path.basename(selected_a_image_path),
        os.path.basename(selected_b_image_path),
    ]
    with open(os.path.join(output_dir, "변화없음.txt"), "a", encoding="utf-8") as f:
        f.write("\n".join(no_change_file_list) + "\n")


# 전역 예외 처리기
def exception_hook(exctype, value, tb):
    tb_msg = ''.join(traceback.format_exception(exctype, value, tb))
    print(tb_msg)
    QMessageBox.critical(None, "오류 발생", tb_msg)
    # 프로그램 종료하지 않음

sys.excepthook = exception_hook

class ImageBox(QWidget):
    def __init__(self, is_after=False):
        super(ImageBox, self).__init__()

        # 이미지가 변화 후 이미지인지 표시하는 플래그
        self.is_after = is_after

        # 라벨 리스트 (두 이미지 간 공유)
        self.poly_list = []

        # 이미지 관련 변수들
        self.path = None  # 원본 이미지 경로
        self.scale = 1.0
        self.w = None
        self.h = None
        self.point = QPoint(0, 0)

        # 원본 이미지
        self.img = None
        self.pair_img = None  # 페어 이미지 (변화 전/후)

        # 상태 플래그들
        self.start_pos = None
        self.end_pos = None
        self.is_left_clicked = False
        self.is_moving = False
        self.setCursor(Qt.PointingHandCursor)
        self.is_drawing = False
        self.line = []
        self.pos = None
        self.is_closed = False

        # 추가 변수들
        self.current_class = 1  # 기본 클래스 번호
        self.selected_poly_index = -1  # 선택된 폴리곤 없음

        # 키보드 이벤트 수신 가능하도록 설정
        self.setFocusPolicy(Qt.ClickFocus)

        # 라인 언두/리두 스택
        self.line_redo_stack = []

        # 페어 이미지 박스 참조 (동기화용)
        self.pair_box = None

    def set_pair_box(self, box):
        """페어 이미지 박스 설정 (동기화용)"""
        self.pair_box = box

    def handle_polygon_class_input(self, pos):
        """
        주어진 위치가 포함된 폴리곤을 찾아 클래스를 입력받는 대화상자를 표시하는 메서드.
        """
        try:
            for index, poly_dict in enumerate(self.poly_list):
                points = poly_dict['points']
                # points의 길이가 짝수인지 확인
                if len(points) < 4 or len(points) % 2 != 0:
                    print(f"폴리곤 {index}의 points 길이가 올바르지 않습니다: {len(points)}")
                    continue

                # 스케일 및 위치 고려한 절대 좌표 계산
                scaled_points = []
                for i in range(0, len(points), 2):
                    x, y = self.get_absolute_coor([[points[i], points[i + 1]]])
                    scaled_points.append(QPoint(int(x), int(y)))

                # QPolygon 생성
                poly = QPolygon(scaled_points)

                # 폴리곤 내부에 주어진 위치가 있는지 확인
                if poly.containsPoint(pos, Qt.OddEvenFill):
                    # 클래스 입력 대화상자 열기
                    class_number, ok = QInputDialog.getInt(
                        self, "클래스 수정", "클래스 번호 입력 (1~6):",
                        value=poly_dict.get('class', 1), min=1, max=6
                    )

                    if ok:
                        # 선택된 폴리곤의 클래스 정보를 업데이트
                        self.poly_list[index]['class'] = class_number

                        # 이미지 라벨 업데이트
                        self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        self.repaint()
                        # 페어 이미지도 업데이트
                        if self.pair_box:
                            self.pair_box.repaint()
                    break  # 탐지된 폴리곤이 처리되었으므로 루프 종료
        except Exception as e:
            print(f"handle_polygon_class_input 오류: {e}")

    def mouseDoubleClickEvent(self, e):
        """
        마우스 더블 클릭 이벤트 처리
        """
        try:
            # 클릭한 좌표를 기준으로 가까운 폴리곤을 찾기
            click_pos = e.pos()
            for index, poly_dict in enumerate(self.poly_list):
                points = poly_dict['points']
                # points의 길이가 짝수인지 확인
                if len(points) < 4 or len(points) % 2 != 0:
                    print(f"폴리곤 {index}의 points 길이가 올바르지 않습니다: {len(points)}")
                    continue

                # 스케일 및 위치 고려한 절대 좌표 계산
                scaled_points = []
                for i in range(0, len(points), 2):
                    x, y = self.get_absolute_coor([[points[i], points[i + 1]]])
                    scaled_points.append(QPoint(int(x), int(y)))

                # QPolygon 생성
                poly = QPolygon(scaled_points)

                # 폴리곤 내부에 더블 클릭된 위치가 있는지 확인
                if poly.containsPoint(click_pos, Qt.OddEvenFill):
                    # 클래스 입력 대화상자 열기
                    class_number, ok = QInputDialog.getInt(self, "클래스 수정", "새 클래스 번호 입력 (1~6):",
                                                           value=poly_dict['class'], min=1, max=6)
                    if ok:
                        # 선택된 폴리곤의 클래스 업데이트
                        self.poly_list[index]['class'] = class_number
                        # 이미지 라벨 업데이트
                        self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        self.repaint()
                        # 페어 이미지도 업데이트
                        if self.pair_box:
                            self.pair_box.repaint()
                    break
        except Exception as e:
            print(f"mouseDoubleClickEvent 오류: {e}")

    def set_image(self):
        """
        이미지를 설정하고 위젯 크기를 이미지 크기에 맞게 조정
        """
        try:
            img_array = np.fromfile(self.path, np.uint8)  # 한글 경로 문제로 우회
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)  # 이미지 디코딩
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # RGB 순서로 변환
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qimg = QImage(img, width, height, bytesPerLine, QImage.Format_RGB888)
            self.img = QPixmap(qimg)

            # 변화 감지 모드가 활성화되어 있었다면 플래그 설정
            if (
                hasattr(self.bigbox, "change_detection_active")
                and self.bigbox.change_detection_active
            ):
                self.bigbox.change_detection_active = False

        except Exception as e:
            print(f"이미지 로드 오류: {e}")
            self.img = QPixmap(self.path)

        try:
            # 스케일 값 초기화
            self.scale = 1.0
            # 이미지 크기 초기화
            self.w = self.img.width()
            self.h = self.img.height()

            # 위젯 크기 설정 - 이미지의 실제 크기로 설정
            self.setMinimumSize(400, 400)
            self.setFixedSize(self.w, self.h)

            self.point = QPoint(0, 0)
            self.repaint()
        except Exception as e:
            print(f"set_image 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 설정 실패: {e}")

    def paintEvent(self, e):
        """
        페인트 이벤트 수신: 이미지, 폴리곤, 미완성 선, 상태 텍스트를 모두 그리기
        """
        try:
            if self.img:
                painter = QPainter()
                painter.begin(self)

                # 이미지 그리기
                painter.drawPixmap(int(self.point.x()), int(self.point.y()), int(self.w), int(self.h), self.img)

                # 변화 감지 모드가 활성화된 상태에서 원본 이미지로 복원시 이미지 다시 로드
                if (
                    hasattr(self.bigbox, "change_detection_active")
                    and not self.bigbox.change_detection_active
                    and self.img
                    and self.path
                ):
                    if hasattr(self, "need_reload") and self.need_reload:
                        self.set_image()
                        self.need_reload = False

                # 완성된 폴리곤 그리기
                for index, poly_dict in enumerate(self.poly_list):
                    points = poly_dict['points']
                    class_number = poly_dict['class']

                    # 클래스 번호에 따른 색상 설정
                    pen = QPen(self.get_class_color(class_number))
                    pen.setWidth(3)

                    # 선택된 폴리곤을 두꺼운 선으로 강조 표시
                    if index == self.selected_poly_index:
                        pen.setWidth(5)  # 선택된 폴리곤 강조

                    painter.setPen(pen)
                    brush = QBrush(self.get_class_color(class_number, alpha=50))
                    painter.setBrush(brush)

                    # points의 길이가 짝수인지 확인
                    if len(points) < 4 or len(points) % 2 != 0:
                        print(f"폴리곤 {index}의 points 길이가 올바르지 않습니다: {len(points)}")
                        continue

                    # 폴리곤 좌표를 QPolygon으로 변환
                    poly = QPolygon()
                    for i in range(0, len(points), 2):
                        x, y = self.get_absolute_coor([[points[i], points[i + 1]]])
                        if np.isnan(x) or np.isnan(y):
                            continue
                        poly.append(QPoint(int(x), int(y)))
                    painter.drawPolygon(poly)

                # 미완성된 선 그리기 (선택된 클래스 색상)
                if self.is_drawing and self.pos and self.line:
                    pen = QPen(self.get_class_color(self.current_class))
                    pen.setWidth(3)
                    painter.setPen(pen)

                    start_x, start_y = self.get_absolute_coor([[self.line[0], self.line[1]]])
                    painter.setBrush(Qt.NoBrush)
                    painter.drawEllipse(QPoint(int(start_x), int(start_y)), 5, 5)  # 시작점 표시

                    # 닫힌 폴리곤을 완성할 경우
                    if self.is_closed:
                        end_x, end_y = self.get_absolute_coor([[self.line[0], self.line[1]]])
                        painter.drawEllipse(QPoint(int(end_x), int(end_y)), 15, 15)

                    # 선 그리기
                    for i in range(0, len(self.line) - 2, 2):
                        x1, y1 = self.get_absolute_coor([[self.line[i], self.line[i + 1]]])
                        x2, y2 = self.get_absolute_coor([[self.line[i + 2], self.line[i + 3]]])
                        if np.isnan(x1) or np.isnan(y1) or np.isnan(x2) or np.isnan(y2):
                            continue
                        painter.drawLine(int(x1), int(y1), int(x2), int(y2))

                    # 마지막 선 그리기 (현재 마우스 위치까지)
                    if self.pos:
                        last_x, last_y = self.get_absolute_coor([[self.line[-2], self.line[-1]]])
                        painter.drawLine(int(last_x), int(last_y), int(self.pos.x()), int(self.pos.y()))

                # 상태 텍스트 그리기 (이미지와 폴리곤 위에)
                font = QFont()
                font.setPointSize(14)
                painter.setFont(font)
                text_x = 10  # 좌측 상단 여백
                text_y = 30  # 텍스트 상단 여백
                if self.is_after:
                    painter.setPen(QPen(Qt.red))
                    painter.drawText(text_x, text_y, "변화 후 (After Change)")
                else:
                    painter.setPen(QPen(Qt.blue))
                    painter.drawText(text_x, text_y, "변화 전 (Before Change)")

                painter.end()

            # 라벨 목록 업데이트
            if hasattr(self, 'bigbox'):
                self.bigbox.set_list()

        except Exception as e:
            print(f"paintEvent 오류: {e}")

    def get_class_color(self, class_number, alpha=255):
        """클래스 번호에 따른 색상 매핑"""
        # 새로운 클래스 색상 매핑
        if class_number == 1:  # 사람
            color = QColor(255, 255, 0, alpha)  # 노란색
        elif class_number == 2:  # 돌
            color = QColor(128, 128, 128, alpha)  # 회색
        elif class_number == 3:  # 흙
            color = QColor(139, 69, 19, alpha)  # 갈색
        elif class_number == 4:  # 물
            color = QColor(0, 0, 255, alpha)  # 파란색
        elif class_number == 5:  # 불
            color = QColor(255, 0, 0, alpha)  # 빨간색
        elif class_number == 6:  # 나무
            color = QColor(0, 128, 0, alpha)  # 초록색
        else:
            color = QColor(0, 0, 0, alpha)  # 기본 검정색
        return color

    def get_absolute_coor(self, coord_list):
        """상대적인 좌표를 절대 좌표로 변환"""
        abs_list = []
        for coor in coord_list:
            x1 = self.point.x() + self.scale * coor[0]
            y1 = self.point.y() + self.scale * coor[1]
            abs_list.extend([x1, y1])
        return abs_list

    def mouseMoveEvent(self, e):
        """
        마우스 이동 이벤트 처리
        """
        try:
            # 이미지 이동
            if self.is_left_clicked:
                self.end_pos = e.pos() - self.start_pos
                old_x, old_y = self.point.x(), self.point.y()
                self.point = self.point + self.end_pos
                self.start_pos = e.pos()
                self.repaint()
                self.is_moving = True

                # 이미지 크기 변경 가능성을 고려하여 setFixedSize 호출
                self.setFixedSize(int(self.w), int(self.h))

                # 페어 이미지 동기화
                if self.pair_box and not self.pair_box.is_left_clicked:
                    dx = self.point.x() - old_x
                    dy = self.point.y() - old_y
                    self.pair_box.point = QPoint(self.pair_box.point.x() + dx, self.pair_box.point.y() + dy)
                    self.pair_box.repaint()

            # 선 그리기 중 마우스 위치 기록
            if self.is_drawing:
                self.pos = e.pos()
                if len(self.line) >= 2:
                    x1 = self.point.x() + self.scale * self.line[0]
                    y1 = self.point.y() + self.scale * self.line[1]
                    if abs(self.pos.x() - x1) < 10 and abs(self.pos.y() - y1) < 10 and len(self.line) > 4:
                        self.is_closed = True
                    else:
                        self.is_closed = False
                self.repaint()
        except Exception as e:
            print(f"mouseMoveEvent 오류: {e}")

    def mousePressEvent(self, e):
        # 플래그 변경
        if e.button() == Qt.LeftButton:
            self.setFocus()
            self.is_left_clicked = True
            self.start_pos = e.pos()

    def mouseReleaseEvent(self, e):
        try:
            # 플래그 변경
            if e.button() == Qt.LeftButton:
                self.is_left_clicked = False
                # 선 또는 점 기록
                if not self.is_moving:
                    # 절대 위치 계산
                    absolute_position = e.pos() - self.point
                    a = absolute_position / self.scale

                    # 선 그리기 시작 또는 종료
                    self.is_drawing = True
                    if self.is_drawing:
                        self.update_line(a)
                self.is_moving = False
            if e.button() == Qt.RightButton and not self.is_moving and self.is_drawing:
                rightMenu = QMenu(self)
                finish_act = QAction(u"완료", self,
                                    triggered=lambda: self.update_line(None, "finish"))
                cancel_act = QAction(u"취소", self,
                                    triggered=lambda: self.update_line(None, "cancel"))
                rightMenu.addAction(finish_act)
                rightMenu.addAction(cancel_act)
                rightMenu.exec_(QCursor.pos())
        except Exception as e:
            print(f"mouseReleaseEvent 오류: {e}")

    def update_line(self, abs, flag="draw"):
        try:
            if flag == "cancel":
                self.line = []
                self.line_redo_stack.clear()
                self.is_drawing = False
                self.repaint()
                self.is_closed = False
            else:
                if flag == "finish":
                    if len(self.line) > 4:
                        # 현재 상태를 undo 스택에 저장
                        self.bigbox.push_undo()
                        self.is_drawing = False
                        self.is_closed = False
                        # 클래스 정보와 함께 폴리곤 저장
                        self.poly_list.append({'points': self.line.copy(), 'class': self.current_class})
                        self.repaint()
                        self.line = []
                        self.line_redo_stack.clear()
                        # 이미지 라벨 업데이트
                        self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        # redo 스택 비우기
                        self.bigbox.redo_stack.clear()

                        # 페어 이미지도 업데이트
                        if self.pair_box:
                            self.pair_box.repaint()

                        # 자동 포인트 추가 기능 해제
                        self.bigbox.auto_adding_points = False
                        self.bigbox.auto_add_timer.stop()
                    else:
                        # 점이 두 개 이하일 때 취소
                        self.update_line(None, "cancel")
                else:
                    if self.is_closed:
                        self.update_line(None, "finish")
                    else:
                        # 점 업데이트
                        self.line.append(abs.x())
                        self.line.append(abs.y())
                        # 점 추가 시 redo 스택 초기화
                        self.line_redo_stack.clear()
            self.repaint()
        except Exception as e:
            print(f"update_line 오류: {e}")
            QMessageBox.warning(self, "오류", f"선 업데이트 실패: {e}")

    def keyPressEvent(self, event):
        """
        키보드 이벤트 처리
        """
        if event.key() == Qt.Key_A:
            # 현재 마우스 위치에 왼쪽 클릭 이벤트 시뮬레이션
            pos = self.mapFromGlobal(QCursor.pos())
            self.is_moving = False
            synthetic_event = QMouseEvent(QEvent.MouseButtonRelease, pos, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
            self.mouseReleaseEvent(synthetic_event)
        elif event.key() == Qt.Key_Z and event.modifiers() & Qt.ControlModifier:
            if self.is_drawing:
                if self.line:
                    self.line_redo_stack.append(self.line[-2:])
                    self.line = self.line[:-2]
                    self.repaint()
            else:
                self.bigbox.undo()
        elif event.key() == Qt.Key_Y and event.modifiers() & Qt.ControlModifier:
            if self.is_drawing:
                if self.line_redo_stack:
                    self.line.extend(self.line_redo_stack.pop())
                    self.repaint()
            else:
                self.bigbox.redo()
        else:
            super().keyPressEvent(event)

    def wheelEvent(self, event):
        # 줌 인/아웃 기능 구현
        try:
            angle = event.angleDelta() / 8
            angleY = angle.y()
            zoom_factor = 1.1
            old_scale = self.scale

            if angleY > 0:
                # 줌 인
                self.scale *= zoom_factor
            else:
                # 줌 아웃
                self.scale /= zoom_factor

            # 이미지 크기 조정
            self.w = self.img.width() * self.scale
            self.h = self.img.height() * self.scale

            # 마우스 포인터 위치를 중심으로 확대/축소
            cursor_pos = event.pos()
            dx = cursor_pos.x() - self.point.x()
            dy = cursor_pos.y() - self.point.y()

            # 새로운 좌표 계산
            new_dx = dx * (self.scale / old_scale)
            new_dy = dy * (self.scale / old_scale)

            # 이미지 위치 조정 (float -> int 변환)
            self.point = QPoint(int(cursor_pos.x() - new_dx), int(cursor_pos.y() - new_dy))

            # 위젯 크기 조정
            self.setFixedSize(int(self.w), int(self.h))

            self.repaint()

            # 페어 이미지 동기화
            if self.pair_box:
                # 같은 스케일 적용
                self.pair_box.scale = self.scale
                self.pair_box.w = self.pair_box.img.width() * self.pair_box.scale
                self.pair_box.h = self.pair_box.img.height() * self.pair_box.scale

                # 상대적인 위치 조정을 위한 비율 계산
                if self.w > 0 and self.pair_box.w > 0:
                    x_ratio = self.point.x() / self.w
                    y_ratio = self.point.y() / self.h

                    # 같은 비율로 위치 적용
                    pair_x = int(x_ratio * self.pair_box.w)
                    pair_y = int(y_ratio * self.pair_box.h)

                    self.pair_box.point = QPoint(pair_x, pair_y)
                else:
                    # 페어 이미지 위치 직접 동기화
                    self.pair_box.point = QPoint(self.point)

                # 페어 이미지 위젯 크기 업데이트
                self.pair_box.setFixedSize(int(self.pair_box.w), int(self.pair_box.h))
                self.pair_box.repaint()

        except Exception as e:
            print(f"wheelEvent 오류: {e}")

class SynchronizedScrollArea(QScrollArea):
    """동기화된 스크롤 영역 클래스"""
    def __init__(self, parent=None):
        super(SynchronizedScrollArea, self).__init__(parent)
        self.pair_scroll = None
        self.isScrolling = False
        
        # 스크롤바 항상 표시
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        
        # 스크롤 영역 속성 설정
        self.setAlignment(Qt.AlignCenter)
        self.setWidgetResizable(False)  # 크기 조정 허용 안함
        
    def set_pair_scroll(self, scroll_area):
        """페어 스크롤 영역 설정"""
        self.pair_scroll = scroll_area
        
    def scrollContentsBy(self, dx, dy):
        """스크롤 동기화"""
        super().scrollContentsBy(dx, dy)
        if self.pair_scroll and not self.pair_scroll.isScrolling:
            self.isScrolling = True
            
            # 스크롤 위치를 상대적으로 맞추기
            h_value = self.horizontalScrollBar().value()
            v_value = self.verticalScrollBar().value()
            
            # 스크롤 범위 비율 계산
            if self.horizontalScrollBar().maximum() > 0 and self.pair_scroll.horizontalScrollBar().maximum() > 0:
                h_ratio = self.pair_scroll.horizontalScrollBar().maximum() / self.horizontalScrollBar().maximum()
                pair_h_value = int(h_value * h_ratio)
                self.pair_scroll.horizontalScrollBar().setValue(pair_h_value)
            
            if self.verticalScrollBar().maximum() > 0 and self.pair_scroll.verticalScrollBar().maximum() > 0:
                v_ratio = self.pair_scroll.verticalScrollBar().maximum() / self.verticalScrollBar().maximum()
                pair_v_value = int(v_value * v_ratio)
                self.pair_scroll.verticalScrollBar().setValue(pair_v_value)
            
            self.isScrolling = False
            
    def wheelEvent(self, event):
        """휠 이벤트 처리 및 동기화"""
        # 확대/축소 비율 조정을 위한 휠 이벤트 처리
        widget = self.widget()
        if widget and isinstance(widget, ImageBox):
            # shift 키 누른 상태에서 휠 사용 시 확대/축소
            modifiers = QApplication.keyboardModifiers()
            if modifiers & Qt.ShiftModifier:
                widget.wheelEvent(event)
                return
                
        # 일반 스크롤
        super().wheelEvent(event)
        
        # 페어 스크롤 동기화
        if self.pair_scroll and not self.pair_scroll.isScrolling:
            self.isScrolling = True
            
            # 상대적인 스크롤 동기화
            if self.horizontalScrollBar().maximum() > 0 and self.pair_scroll.horizontalScrollBar().maximum() > 0:
                h_ratio = self.pair_scroll.horizontalScrollBar().maximum() / self.horizontalScrollBar().maximum()
                h_value = self.horizontalScrollBar().value()
                pair_h_value = int(h_value * h_ratio)
                self.pair_scroll.horizontalScrollBar().setValue(pair_h_value)
            
            if self.verticalScrollBar().maximum() > 0 and self.pair_scroll.verticalScrollBar().maximum() > 0:
                v_ratio = self.pair_scroll.verticalScrollBar().maximum() / self.verticalScrollBar().maximum()
                v_value = self.verticalScrollBar().value()
                pair_v_value = int(v_value * v_ratio)
                self.pair_scroll.verticalScrollBar().setValue(pair_v_value)
                
            self.isScrolling = False

class change_detection(QMainWindow):
    def __init__(self, parent=None):
        super(change_detection, self).__init__(parent)
        self.temp_listA = []
        self.temp_listB = []
        self.image_labels = {}  # 이미지별 라벨 저장 딕셔너리

        # Undo 및 Redo 스택
        self.undo_stack = []
        self.redo_stack = []

        # 선택된 이미지 경로를 저장하는 변수 추가
        self.selected_a_image_path = None  # Before 이미지 경로
        self.selected_b_image_path = None  # After 이미지 경로

        # 픽셀 변화 감지 관련 변수 추가
        self.change_detection_active = False  # 픽셀 변화 감지 모드 활성화 여부
        self.change_mask = None  # 변화 감지 마스크
        self.change_threshold = 30  # 기본 임계값

        # 윈도우 크기 설정
        self.resize(int(1400*0.8), int(1100*0.8))
        # QTimer 설정: 단일 클릭과 더블 클릭 구분
        self.click_timer = QTimer()
        self.click_timer.setSingleShot(True)
        self.click_timer.timeout.connect(self.handle_single_click)

        # 메뉴 설정
        importAct = QAction('가져오기', self, triggered=self.openimage)
        saveAct = QAction('저장', self, triggered=self.savepoint)
        saveAct.setShortcut('Ctrl+S')
        undoAct = QAction('실행 취소', self, triggered=self.undo)
        redoAct = QAction('다시 실행', self, triggered=self.redo)
        exitAct = QAction('종료', self)
        exitAct.setShortcut('Ctrl+Q')
        exitAct.triggered.connect(self.close)

        bar = self.menuBar()
        file = bar.addMenu("파일")

        file.addActions([importAct, saveAct, undoAct, redoAct, exitAct])

        # 웹 브라우저에서 URL을 여는 QAction
        url_act = QAction("URL : https://github.com/chartgod/Changedetection_labelingtool", self)
        url_act.triggered.connect(lambda: webbrowser.open("https://github.com/chartgod/Changedetection_labelingtool"))

        # 메인 레이아웃 설정
        main_layout = QHBoxLayout()

        # 동기화 비교 뷰 설정
        self.comparison_widget = QWidget()
        comparison_layout = QHBoxLayout(self.comparison_widget)

        # 변화 전 이미지 (before)
        self.before_scroll = SynchronizedScrollArea()
        self.before_box = ImageBox(is_after=False)
        self.before_box.setMouseTracking(True)
        self.before_box.bigbox = self
        self.before_scroll.setWidget(self.before_box)
        self.before_scroll.isScrolling = False

        # 변화 후 이미지 (after)
        self.after_scroll = SynchronizedScrollArea()
        self.after_box = ImageBox(is_after=True)
        self.after_box.setMouseTracking(True)
        self.after_box.bigbox = self
        self.after_scroll.setWidget(self.after_box)
        self.after_scroll.isScrolling = False

        # 스크롤 영역 동기화 설정
        self.before_scroll.set_pair_scroll(self.after_scroll)
        self.after_scroll.set_pair_scroll(self.before_scroll)
        self.before_scroll.isScrolling = False
        self.after_scroll.isScrolling = False

        # 이미지 박스 동기화 설정
        self.before_box.set_pair_box(self.after_box)
        self.after_box.set_pair_box(self.before_box)

        # 비교 뷰에 이미지 추가
        comparison_layout.addWidget(self.before_scroll)
        comparison_layout.addWidget(self.after_scroll)

        # 라벨 리스트
        self.LV_label = QListView()
        self.LV_label.setContextMenuPolicy(Qt.CustomContextMenu)
        self.LV_label.customContextMenuRequested.connect(self.rightMenuShow2)

        # 더블 클릭 시 select_polygon 호출
        # 클릭 이벤트와 더블 클릭 이벤트 연결
        self.LV_label.clicked.connect(self.start_single_click_timer)
        self.LV_label.doubleClicked.connect(self.handle_double_click)

        # 파일 리스트
        self.LV_A = QListView()
        self.LV_B = QListView()

        # 버튼 및 입력창
        self.import_btn = QPushButton("가져오기...")
        self.import_btnB = QPushButton("가져오기...")
        self.label_a = QLabel("변화 전(Before)")
        self.label_b = QLabel("변화 후(After)")
        self.save_btn = QPushButton("라벨 저장")
        self.no_change_save_btn = QPushButton("변화 없음 라벨 저장")
        self.class_input_label = QLabel('클래스:')
        self.class_input = QLineEdit()
        self.class_input.setText('1')
        self.class_input.setValidator(QIntValidator(1, 6))  # 클래스 1~6 허용

        # 변화 감지 버튼 추가
        self.detect_changes_btn = QPushButton("단순픽셀 변화감지")
        self.detect_changes_btn.clicked.connect(self.detect_pixel_changes)
        self.reset_images_btn = QPushButton("원본 이미지로 복원")
        self.reset_images_btn.clicked.connect(self.reset_images)

        # 상태 변수들
        self.auto_switching = False
        self.auto_switch_interval = 1000  # 기본 간격(ms)

        # 추가된 상태 변수
        self.auto_adding_points = False
        self.auto_add_interval = 200  # milliseconds
        self.auto_add_timer = QTimer()
        self.auto_add_timer.timeout.connect(self.add_point_at_cursor)

        # 시그널 및 슬롯 연결
        self.setWindowTitle("변화 감지 라벨링 도구")
        self.import_btn.clicked.connect(lambda: self.openimage("A"))
        self.import_btnB.clicked.connect(lambda: self.openimage("B"))
        self.LV_A.clicked.connect(self.load_image_pair)
        self.LV_B.clicked.connect(self.load_image_pair)
        self.save_btn.clicked.connect(self.savepoint)
        self.no_change_save_btn.clicked.connect(self.save_no_change)
        self.class_input.returnPressed.connect(self.update_class)

        # 레이아웃 설정
        V_Tool = QVBoxLayout()
        H_TempBox = QHBoxLayout()

        # 도구 영역
        # V_Tool.addWidget(self.save_btn)
        save_buttons_layout = QHBoxLayout()
        save_buttons_layout.addWidget(self.save_btn)
        save_buttons_layout.addWidget(self.no_change_save_btn)
        V_Tool.addLayout(save_buttons_layout)

        # 클래스 버튼들 가로 배치
        H_ClassButtons = QHBoxLayout()

        # 클래스 버튼 추가
        self.person_btn = QPushButton("사람")
        self.person_btn.clicked.connect(lambda: self.update_class_from_button(1))
        H_ClassButtons.addWidget(self.person_btn)

        self.stone_btn = QPushButton("돌")
        self.stone_btn.clicked.connect(lambda: self.update_class_from_button(2))
        H_ClassButtons.addWidget(self.stone_btn)

        self.soil_btn = QPushButton("흙")
        self.soil_btn.clicked.connect(lambda: self.update_class_from_button(3))
        H_ClassButtons.addWidget(self.soil_btn)

        self.water_btn = QPushButton("물")
        self.water_btn.clicked.connect(lambda: self.update_class_from_button(4))
        H_ClassButtons.addWidget(self.water_btn)

        self.fire_btn = QPushButton("불")
        self.fire_btn.clicked.connect(lambda: self.update_class_from_button(5))
        H_ClassButtons.addWidget(self.fire_btn)

        self.tree_btn = QPushButton("나무")
        self.tree_btn.clicked.connect(lambda: self.update_class_from_button(6))
        H_ClassButtons.addWidget(self.tree_btn)

        V_Tool.addLayout(H_ClassButtons)

        # 변화 감지 섹션 제목 추가
        change_detection_label = QLabel("변화 감지 옵션")
        change_detection_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        V_Tool.addWidget(change_detection_label)

        # 임계값 슬라이더 추가
        threshold_layout = QHBoxLayout()
        threshold_label = QLabel("감지 민감도:")
        self.threshold_slider = QSlider(Qt.Horizontal)
        self.threshold_slider.setMinimum(5)
        self.threshold_slider.setMaximum(100)
        self.threshold_slider.setValue(self.change_threshold)
        self.threshold_slider.setToolTip("값이 낮을수록 작은 변화도 감지합니다.")
        self.threshold_value_label = QLabel(f"{self.change_threshold}")

        # 슬라이더 값 변경 시 라벨 업데이트
        self.threshold_slider.valueChanged.connect(self.update_threshold_value)

        # 레이아웃에 추가
        threshold_layout.addWidget(threshold_label)
        threshold_layout.addWidget(self.threshold_slider)
        threshold_layout.addWidget(self.threshold_value_label)
        V_Tool.addLayout(threshold_layout)

        # 변화 감지 버튼 레이아웃
        change_detection_btns = QHBoxLayout()
        change_detection_btns.addWidget(self.detect_changes_btn)
        change_detection_btns.addWidget(self.reset_images_btn)
        V_Tool.addLayout(change_detection_btns)

        # 버튼 스타일 설정
        self.detect_changes_btn.setStyleSheet(
            """
            QPushButton {
                background-color: #a970ff;
                color: white;
                font-weight: bold;
                padding: 5px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #8a50e8;
            }
        """
        )
        self.reset_images_btn.setStyleSheet(
            """
            QPushButton {
                background-color: #555555;
                color: white;
                padding: 5px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #777777;
            }
        """
        )

        # 툴팁 추가
        self.detect_changes_btn.setToolTip(
            "두 이미지 간의 픽셀 차이를 보라색으로 표시합니다. 양쪽에 동일한 변화 후 이미지가 보여집니다."
        )
        self.reset_images_btn.setToolTip(
            "변화 감지 모드를 해제하고 원래 이미지 상태로 돌아갑니다."
        )

        # 버튼 크기 조정
        self.detect_changes_btn.setMinimumHeight(30)
        self.reset_images_btn.setMinimumHeight(30)

        V_Tool.addWidget(self.class_input_label)
        V_Tool.addWidget(self.class_input)
        V_Tool.addLayout(H_TempBox)

        TempA = QVBoxLayout()
        TempA.addWidget(self.import_btn)
        TempA.addWidget(self.label_a)
        TempA.addWidget(self.LV_A)
        TempB = QVBoxLayout()
        TempB.addWidget(self.import_btnB)
        TempB.addWidget(self.label_b)
        TempB.addWidget(self.LV_B)
        H_TempBox.addLayout(TempA)
        H_TempBox.addLayout(TempB)

        # 오른쪽 패널 구성 (도구 + 라벨 리스트)
        V_RightPanel = QVBoxLayout()
        V_RightPanel.addLayout(V_Tool)
        V_RightPanel.addWidget(self.LV_label)

        right_widget = QWidget()
        right_widget.setLayout(V_RightPanel)

        # 스플리터 설정
        H_Splitter = QSplitter(Qt.Horizontal)
        H_Splitter.addWidget(self.comparison_widget)
        H_Splitter.addWidget(right_widget)
        H_Splitter.setStretchFactor(0, 3)
        H_Splitter.setStretchFactor(1, 1)
        H_Splitter.setSizes([800, 200])

        main_layout.addWidget(H_Splitter)
        main_frame = QWidget()
        main_frame.setLayout(main_layout)
        self.setCentralWidget(main_frame)

        # 메인 윈도우가 키보드 이벤트를 받을 수 있도록 설정
        self.setFocusPolicy(Qt.StrongFocus)
        self.showMaximized()
        self.set_list()

    def update_threshold_value(self, value):
        """
        임계값 슬라이더 값 변경 시 호출되는 함수
        """
        self.change_threshold = value
        self.threshold_value_label.setText(f"{value}")

    def detect_pixel_changes(self):
        """
        두 이미지 간의 픽셀 변화를 감지하여 보라색으로 표시합니다.
        변화 전/후 이미지 모두 변화 후 이미지를 보여주고, 변화된 부분만 보라색으로 강조합니다.
        """
        try:
            if not self.selected_a_image_path or not self.selected_b_image_path:
                QMessageBox.information(self, "경고", "이미지가 선택되지 않았습니다.")
                return

            # 변화 전 이미지 로드
            img_array_a = np.fromfile(self.selected_a_image_path, np.uint8)
            img_a = cv2.imdecode(img_array_a, cv2.IMREAD_COLOR)

            # 변화 후 이미지 로드
            img_array_b = np.fromfile(self.selected_b_image_path, np.uint8)
            img_b = cv2.imdecode(img_array_b, cv2.IMREAD_COLOR)

            if img_a is None or img_b is None:
                QMessageBox.warning(self, "오류", "이미지 로드에 실패했습니다.")
                return

            # 이미지 크기가 다른 경우 크기 맞추기
            if img_a.shape != img_b.shape:
                img_b = cv2.resize(img_b, (img_a.shape[1], img_a.shape[0]))

            # 이미지 간의 차이 계산
            diff = cv2.absdiff(img_a, img_b)

            # 특정 임계값보다 큰 변화만 감지하기 위한 임계값 설정
            threshold = self.change_threshold

            # 각 채널에서 임계값보다 큰 차이가 있는지 확인
            mask = np.any(diff > threshold, axis=2).astype(np.uint8) * 255

            # 노이즈 제거를 위한 모폴로지 연산 적용
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 변화가 감지된 마스크 저장
            self.change_mask = mask.copy()

            # 변화 후 이미지에 보라색으로 변화 표시
            img_b_with_changes = img_b.copy()
            img_b_with_changes[mask > 0] = (180, 0, 180)  # 보라색 (BGR)

            # 이미지를 화면에 표시하기 위해 RGB로 변환
            img_b_with_changes = cv2.cvtColor(img_b_with_changes, cv2.COLOR_BGR2RGB)

            # 이미지 크기 정보
            height, width, channel = img_b_with_changes.shape
            bytesPerLine = 3 * width

            # 변화가 표시된 이미지를 QImage로 변환 (양쪽 모두 같은 이미지)
            qimg = QImage(
                img_b_with_changes, width, height, bytesPerLine, QImage.Format_RGB888
            )

            # QImage를 QPixmap으로 변환하여 화면에 표시 (양쪽 모두 변화 후 이미지 + 변화 표시)
            result_pixmap = QPixmap(qimg)

            # 원본 이미지 저장 (복원용)
            if not hasattr(self, "original_before_img"):
                self.original_before_img = self.before_box.img.copy()
                self.original_after_img = self.after_box.img.copy()

            # 양쪽 이미지 박스에 변화 감지 이미지 설정
            self.before_box.img = result_pixmap
            self.after_box.img = result_pixmap

            # 변화 감지 모드 활성화 플래그 설정
            self.change_detection_active = True

            # 이미지 업데이트
            self.before_box.repaint()
            self.after_box.repaint()

            QMessageBox.information(
                self,
                "픽셀 변화 감지",
                "이미지 간 변화가 보라색으로 표시되었습니다. 양쪽 화면에 '변화 후' 이미지가 표시됩니다.",
            )

        except Exception as e:
            print(f"detect_pixel_changes 오류: {e}")
            QMessageBox.warning(self, "오류", f"픽셀 변화 감지 실패: {e}")

    def reset_images(self):
        """
        변화 감지 모드를 해제하고 원본 이미지로 돌아갑니다.
        """
        try:
            if (
                hasattr(self, "change_detection_active")
                and self.change_detection_active
            ):
                # 저장된 원본 이미지가 있으면 복원
                if hasattr(self, "original_before_img") and hasattr(
                    self, "original_after_img"
                ):
                    # 원본 이미지 복원
                    self.before_box.img = self.original_before_img
                    self.after_box.img = self.original_after_img

                    # 변수 정리
                    delattr(self, "original_before_img")
                    delattr(self, "original_after_img")
                else:
                    # 저장된 원본 이미지가 없으면 경로에서 다시 로드
                    self.before_box.set_image()
                    self.after_box.set_image()

                # 변화 감지 모드 비활성화
                self.change_detection_active = False
                self.change_mask = None

                # 이미지 업데이트
                self.before_box.repaint()
                self.after_box.repaint()

                QMessageBox.information(
                    self, "원본 복원", "원본 이미지로 복원되었습니다."
                )

        except Exception as e:
            print(f"reset_images 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 초기화 실패: {e}")

        """
        변화 없음 라벨을 저장하는 메서드
        binary_cd 및 semantic_cd 폴더에 검정색 이미지를 저장합니다.
        변화 감지 마스크가 있는 경우 해당 영역은 제외합니다.
        """
        try:
            if not self.selected_a_image_path or not self.selected_b_image_path:
                QMessageBox.information(
                    self, "경고", "저장할 이미지가 선택되지 않았습니다."
                )
                return

            # 'binary_cd'와 'semantic_cd' 폴더 생성
            current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
            binary_dir = os.path.join(current_dir, "binary_cd")
            semantic_dir = os.path.join(current_dir, "semantic_cd")

            if not os.path.exists(binary_dir):
                os.makedirs(binary_dir)
            if not os.path.exists(semantic_dir):
                os.makedirs(semantic_dir)

            # before와 after 이미지 모두 처리
            image_paths = [self.selected_a_image_path, self.selected_b_image_path]

            for image_path in image_paths:
                # 원본 이미지 로드하여 크기 얻기
                img_array = np.fromfile(image_path, np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if img is None:
                    print(f"이미지 로드 실패: {image_path}")
                    continue

                height, width = img.shape[:2]

                # 검은색 이미지 생성 (RGB 값을 0으로 설정)
                black_image = np.zeros((height, width, 3), dtype=np.uint8)

                # 변화 감지 마스크가 있는 경우 해당 영역 제외
                if hasattr(self, "change_mask") and self.change_mask is not None:
                    # 변화 마스크 크기 조정
                    if self.change_mask.shape[:2] != (height, width):
                        change_mask_resized = cv2.resize(
                            self.change_mask, (width, height)
                        )
                    else:
                        change_mask_resized = self.change_mask

                    # 변화 감지 영역은 저장하지 않음 (이미 검은색이므로 추가 처리 필요 없음)
                    # 필요하다면 여기에 특별한 표시 코드를 추가할 수 있음

                # 파일명 준비
                (filepath, filename) = os.path.split(image_path)
                base_filename = os.path.splitext(filename)[0]

                # 이미지 저장
                binary_filename = os.path.join(binary_dir, f"{base_filename}.jpg")
                semantic_filename = os.path.join(semantic_dir, f"{base_filename}.jpg")

                cv2.imwrite(binary_filename, black_image)
                cv2.imwrite(semantic_filename, black_image)

            QMessageBox.information(
                self, "저장", "변화 없음 라벨이 성공적으로 저장되었습니다."
            )
        except Exception as e:
            print(f"save_no_change 오류: {e}")
            QMessageBox.warning(self, "오류", f"변화 없음 라벨 저장 실패: {e}")

    def save_no_change(self):
        """
        변화 없음 라벨을 저장하는 메서드
        binary_cd 및 semantic_cd 폴더에 검정색 이미지를 저장합니다.
        """
        try:
            if not self.selected_a_image_path or not self.selected_b_image_path:
                QMessageBox.information(self, "경고", "저장할 이미지가 선택되지 않았습니다.")
                return

            # 'binary_cd'와 'semantic_cd' 폴더 생성
            current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
            binary_dir = os.path.join(current_dir, 'binary_cd')
            semantic_dir = os.path.join(current_dir, 'semantic_cd')

            if not os.path.exists(binary_dir):
                os.makedirs(binary_dir)
            if not os.path.exists(semantic_dir):
                os.makedirs(semantic_dir)

            # before와 after 이미지 모두 처리
            image_paths = [self.selected_a_image_path, self.selected_b_image_path]

            for image_path in image_paths:
                # 원본 이미지 로드하여 크기 얻기
                img_array = np.fromfile(image_path, np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if img is None:
                    print(f"이미지 로드 실패: {image_path}")
                    continue

                height, width = img.shape[:2]

                # 검은색 이미지 생성 (RGB 값을 0으로 설정)
                black_image = np.zeros((height, width, 3), dtype=np.uint8)

                # 파일명 준비
                (filepath, filename) = os.path.split(image_path)
                base_filename = os.path.splitext(filename)[0]

                # 이미지 저장
                binary_filename = os.path.join(binary_dir, f"{base_filename}.jpg")
                semantic_filename = os.path.join(semantic_dir, f"{base_filename}.jpg")

                cv2.imwrite(binary_filename, black_image)
                cv2.imwrite(semantic_filename, black_image)

            QMessageBox.information(self, "저장", "변화 없음 라벨이 성공적으로 저장되었습니다.")
            append_no_change_file(
                os.path.join(current_dir, "label_stats"),
                self.selected_a_image_path,
                self.selected_b_image_path,
            )
        except Exception as e:
            print(f"save_no_change 오류: {e}")
            QMessageBox.warning(self, "오류", f"변화 없음 라벨 저장 실패: {e}")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W:
            # 자동 포인트 지정 기능 토글
            self.auto_adding_points = not self.auto_adding_points
            if self.auto_adding_points:
                # 타이머 시작
                self.auto_add_timer.start(self.auto_add_interval)
            else:
                # 타이머 중지
                self.auto_add_timer.stop()
        elif event.key() == Qt.Key_A:
            # 현재 커서 위치에 왼쪽 클릭 이벤트 시뮬레이션
            # 현재 포커스된 이미지 박스에 이벤트 전달
            focused_widget = QApplication.focusWidget()
            if isinstance(focused_widget, ImageBox):
                pos = focused_widget.mapFromGlobal(QCursor.pos())
                focused_widget.is_moving = False
                synthetic_event = QMouseEvent(QEvent.MouseButtonRelease, pos, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
                focused_widget.mouseReleaseEvent(synthetic_event)
        # 숫자 1~6번 키에 따른 클래스 변경 추가
        elif Qt.Key_1 <= event.key() <= Qt.Key_6:
            self.update_class_from_button(event.key() - Qt.Key_0)  # 키코드에서 숫자 추출
        elif event.key() == Qt.Key_Z and event.modifiers() & Qt.ControlModifier:
            self.undo()
        elif event.key() == Qt.Key_Y and event.modifiers() & Qt.ControlModifier:
            self.redo()
        else:
            # 이벤트 전달
            focused_widget = QApplication.focusWidget()
            if isinstance(focused_widget, ImageBox):
                focused_widget.keyPressEvent(event)
            else:
                super().keyPressEvent(event)

    def add_point_at_cursor(self):
        # 현재 커서 위치에 포인트 추가
        # 현재 포커스된 이미지 박스에 이벤트 전달
        focused_widget = QApplication.focusWidget()
        if isinstance(focused_widget, ImageBox):
            pos = focused_widget.mapFromGlobal(QCursor.pos())
            focused_widget.is_moving = False
            synthetic_event = QMouseEvent(QEvent.MouseButtonRelease, pos, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
            focused_widget.mouseReleaseEvent(synthetic_event)
            # Undo 스택에 상태 저장
            self.push_undo()

    def update_class_from_button(self, class_number):
        """
        버튼 클릭 시 해당 클래스 번호로 설정하는 함수
        """
        if 1 <= class_number <= 6:
            self.before_box.current_class = class_number
            self.after_box.current_class = class_number
            self.class_input.setText(str(class_number))

    def update_class(self):
        try:
            class_number = int(self.class_input.text())
            if 1 <= class_number <= 6:
                self.before_box.current_class = class_number
                self.after_box.current_class = class_number
        except Exception as e:
            print(f"update_class 오류: {e}")
            QMessageBox.warning(self, "오류", f"유효하지 않은 클래스 번호: {e}")

    def start_single_click_timer(self, index):
        # 단일 클릭의 index를 저장하고 타이머 시작
        self.click_index = index
        self.click_timer.start(200)  # 200ms 후에 단일 클릭으로 간주

    def handle_single_click(self):
        # 단일 클릭 시 폴리곤을 강조 표시
        self.select_polygon(self.click_index, double_click=False)

    def handle_double_click(self, index):
        # 더블 클릭 시 폴리곤 클래스 입력 창 표시
        self.click_timer.stop()  # 단일 클릭 타이머 취소
        self.select_polygon(index, double_click=True)

    def select_polygon(self, index, double_click=False):
        """
        폴리곤 항목을 클릭하여 강조 표시하고, 더블 클릭 시 클래스 번호 수정 창 표시
        """
        try:
            poly_index = index.row()

            # 선택된 폴리곤 인덱스를 저장하여 강조 표시
            self.before_box.selected_poly_index = poly_index
            self.after_box.selected_poly_index = poly_index
            self.before_box.repaint()
            self.after_box.repaint()

            if double_click:
                # 더블 클릭 시 클래스 번호 입력 창 표시
                class_number, ok = QInputDialog.getInt(
                    self, "클래스 수정", "클래스 번호 입력 (1~6):", 
                    value=self.before_box.poly_list[poly_index].get('class', 1), 
                    min=1, max=6
                )

                if ok:
                    # 선택된 폴리곤의 클래스 정보를 업데이트
                    self.before_box.poly_list[poly_index]['class'] = class_number

                    # 이미지 라벨 업데이트
                    self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()

                    # 화면 업데이트 및 폴리곤 강조 표시
                    self.before_box.repaint()
                    self.after_box.repaint()
                    self.set_list()

        except Exception as e:
            print(f"select_polygon 오류: {e}")
            QMessageBox.warning(self, "오류", f"클래스 수정 실패: {e}")

    def rightMenuShow2(self, point):
        rightMenu = QMenu(self.LV_label)
        removeAction = QAction(u"삭제", self, triggered=self.removepoint)
        rightMenu.addAction(removeAction)
        rightMenu.exec_(self.LV_label.mapToGlobal(point))

    def push_undo(self, clear_redo=True):
        # 현재 상태를 undo 스택에 저장
        self.undo_stack.append(self.before_box.poly_list.copy())
        # redo 스택 비우기 (clear_redo가 True일 때만)
        if clear_redo:
            self.redo_stack.clear()

    def removepoint(self):
        try:
            selected = self.LV_label.selectedIndexes()
            if not selected:
                return
            # 현재 상태를 undo 스택에 저장
            self.push_undo()
            for i in selected:
                self.before_box.poly_list.pop(i.row())
            self.before_box.selected_poly_index = -1  # 선택 인덱스 리셋
            self.after_box.selected_poly_index = -1
            self.before_box.repaint()
            self.after_box.repaint()
            # 이미지 라벨 업데이트
            self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()
        except Exception as e:
            print(f"removepoint 오류: {e}")
            QMessageBox.warning(self, "오류", f"포인트 삭제 실패: {e}")

    def undo(self):
        try:
            if not self.undo_stack:
                QMessageBox.information(self, "실행 취소", "되돌릴 작업이 없습니다.")
                return
            # 현재 상태를 redo 스택에 저장
            self.redo_stack.append(self.before_box.poly_list.copy())
            # undo 스택에서 상태 복원
            self.before_box.poly_list = self.undo_stack.pop()
            self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()
            self.before_box.selected_poly_index = -1  # 선택 인덱스 리셋
            self.after_box.selected_poly_index = -1
            self.before_box.repaint()
            self.after_box.repaint()
        except Exception as e:
            print(f"undo 오류: {e}")
            QMessageBox.warning(self, "오류", f"실행 취소 실패: {e}")

    def redo(self):
        try:
            if not self.redo_stack:
                QMessageBox.information(self, "다시 실행", "다시 실행할 작업이 없습니다.")
                return
            # 현재 상태를 undo 스택에 저장, redo 시에는 redo_stack을 비우지 않음
            self.push_undo(clear_redo=False)
            # redo 스택에서 상태 복원
            self.before_box.poly_list = self.redo_stack.pop()
            self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()
            self.before_box.selected_poly_index = -1  # 선택 인덱스 리셋
            self.after_box.selected_poly_index = -1
            self.before_box.repaint()
            self.after_box.repaint()
        except Exception as e:
            print(f"redo 오류: {e}")
            QMessageBox.warning(self, "오류", f"다시 실행 실패: {e}")

    def load_image_pair(self, qModelIndex):
        try:
            index = qModelIndex.row()
            # 인덱스가 두 리스트의 범위 내에 있는지 확인
            if index < len(self.temp_listA) and index < len(self.temp_listB):
                # 현재 라벨 저장
                if self.before_box.path:
                    self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()
                # undo 및 redo 스택 초기화
                self.undo_stack.clear()
                self.redo_stack.clear()

                # 새로운 이미지 경로 설정
                self.selected_a_image_path = self.temp_listA[index]
                self.selected_b_image_path = self.temp_listB[index]

                # 변화 전 이미지(Before) 설정
                self.before_box.path = self.selected_a_image_path
                self.before_box.set_image()

                # 변화 후 이미지(After) 설정
                self.after_box.path = self.selected_b_image_path
                self.after_box.set_image()

                # 새로운 이미지에 대한 라벨 초기화
                if self.before_box.path in self.image_labels:
                    self.before_box.poly_list = self.image_labels[self.before_box.path].copy()  # 기존 라벨이 있을 경우 복원
                    self.after_box.poly_list = self.before_box.poly_list  # 두 이미지가 동일한 폴리곤 리스트 공유
                else:
                    self.before_box.poly_list = []  # 기존 라벨이 없을 경우 초기화
                    self.after_box.poly_list = self.before_box.poly_list  # 두 이미지가 동일한 폴리곤 리스트 공유
                    self.load_labels_from_file()

                # 변화 감지 모드 초기화
                self.change_detection_active = False
                self.change_mask = None

                self.before_box.repaint()
                self.after_box.repaint()
                self.set_list()
            else:
                QMessageBox.warning(self, "오류", "변화 전 이미지와 변화 후 이미지의 수가 다릅니다.")
        except Exception as e:
            print(f"load_image_pair 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 로드 실패: {e}")

    def openimage(self, flag):
        try:
            if self.before_box.path:
                self.image_labels[self.before_box.path] = (
                    self.before_box.poly_list.copy()
                )
            self.undo_stack.clear()
            self.redo_stack.clear()

            # 폴더 선택
            folder_path = QFileDialog.getExistingDirectory(self, "폴더 선택", "")
            if not folder_path:
                return

            # 이미지 파일 확장자 필터링
            image_extensions = (".png", ".jpg", ".jpeg", ".bmp")
            image_files = [
                os.path.join(folder_path, f)
                for f in os.listdir(folder_path)
                if f.lower().endswith(image_extensions)
            ]
            image_files.sort(key=lambda x: natural_key(os.path.basename(x)))

            if flag == "A":
                self.temp_listA = image_files
            else:
                self.temp_listB = image_files

            self.set_list()

            # 변화 전/후 이미지 수 확인 후 첫 쌍 불러오기
            if self.temp_listA and self.temp_listB:
                if len(self.temp_listA) != len(self.temp_listB):
                    QMessageBox.warning(
                        self, "오류", "변화 전 이미지와 변화 후 이미지의 수가 다릅니다."
                    )
                else:
                    self.selected_a_image_path = self.temp_listA[0]
                    self.selected_b_image_path = self.temp_listB[0]
                    self.before_box.path = self.selected_a_image_path
                    self.after_box.path = self.selected_b_image_path
                    self.before_box.set_image()
                    self.after_box.set_image()
                    if self.before_box.path in self.image_labels:
                        self.before_box.poly_list = self.image_labels[self.before_box.path].copy()
                        self.after_box.poly_list = self.before_box.poly_list
                    else:
                        self.before_box.poly_list = []
                        self.after_box.poly_list = self.before_box.poly_list
                        self.load_labels_from_file()
                    self.before_box.repaint()
                    self.after_box.repaint()
                    self.set_list()
        except Exception as e:
            print(f"openimage 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 열기 실패: {e}")

    def savepoint(self):
        try:
            if not self.image_labels:
                QMessageBox.information(self, "경고", "저장할 레이블이 없습니다.")
                return

            # 'binary_cd'와 'semantic_cd' 폴더 생성
            current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
            binary_dir = os.path.join(current_dir, 'binary_cd')
            semantic_dir = os.path.join(current_dir, 'semantic_cd')

            if not os.path.exists(binary_dir):
                os.makedirs(binary_dir)
            if not os.path.exists(semantic_dir):
                os.makedirs(semantic_dir)

            for image_path in self.image_labels.keys():
                poly_list = self.image_labels.get(image_path, [])
                if not poly_list:
                    continue

                # 원본 이미지 로드
                img_array = np.fromfile(image_path, np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if img is None:
                    print(f"이미지 로드 실패: {image_path}")
                    continue

                height, width = img.shape[:2]

                # 바이너리 CD 이미지 생성 (흰색 폴리곤)
                binary_mask = np.zeros((height, width), dtype=np.uint8)

                # 시맨틱 CD 이미지 생성 (클래스별 색상)
                semantic_mask = np.zeros((height, width, 3), dtype=np.uint8)

                # 검은색 배경 생성 (RGB 값을 0으로 설정)
                binary_result = np.zeros((height, width, 3), dtype=np.uint8)
                semantic_result = np.zeros((height, width, 3), dtype=np.uint8)

                # 각 폴리곤 그리기
                for poly_dict in poly_list:
                    points = poly_dict['points']
                    class_number = poly_dict['class']

                    if len(points) < 4 or len(points) % 2 != 0:
                        continue

                    # 폴리곤 포인트 변환
                    polygon_points = []
                    for i in range(0, len(points), 2):
                        polygon_points.append([int(points[i]), int(points[i+1])])
                    polygon_points = np.array([polygon_points], dtype=np.int32)

                    # 바이너리 마스크에 폴리곤 채우기 (흰색)
                    cv2.fillPoly(binary_mask, polygon_points, 255)

                    # 시맨틱 마스크에 클래스별 색상으로 폴리곤 채우기
                    if class_number == 1:  # 사람 - 노란색
                        color = (0, 255, 255)  # BGR
                    elif class_number == 2:  # 돌 - 회색
                        color = (128, 128, 128)
                    elif class_number == 3:  # 흙 - 갈색
                        color = (19, 69, 139)
                    elif class_number == 4:  # 물 - 파란색
                        color = (255, 0, 0)
                    elif class_number == 5:  # 불 - 빨간색
                        color = (0, 0, 255)
                    elif class_number == 6:  # 나무 - 초록색
                        color = (0, 128, 0)
                    else:
                        color = (0, 0, 0)

                    cv2.fillPoly(semantic_mask, polygon_points, color)

                # 변화 감지 마스크 생성 - 폴리곤 외부 영역에만 적용
                change_mask_outside_polygons = None
                if hasattr(self, "change_mask") and self.change_mask is not None:
                    # 변화 마스크 크기 조정
                    if self.change_mask.shape[:2] != (height, width):
                        change_mask_resized = cv2.resize(
                            self.change_mask, (width, height)
                        )
                    else:
                        change_mask_resized = self.change_mask

                    # 폴리곤 외부 영역에만 변화 감지 마스크 적용
                    change_mask_outside_polygons = change_mask_resized.copy()
                    change_mask_outside_polygons[binary_mask == 255] = (
                        0  # 폴리곤 내부는 제외
                    )

                # 바이너리 이미지 저장 (폴리곤 영역만 흰색, 나머지는 검은색)
                binary_result[binary_mask == 255] = (255, 255, 255)  # 흰색으로 표시

                # 시맨틱 이미지 저장 (폴리곤 영역만 컬러, 나머지는 검은색)
                semantic_result = np.zeros_like(semantic_mask)  # 결과를 0으로 초기화

                # 폴리곤 영역을 그대로 추가
                mask_polygons = (semantic_mask > 0).any(axis=2)
                semantic_result[mask_polygons] = semantic_mask[mask_polygons]

                # 파일명 준비
                (filepath, filename) = os.path.split(image_path)
                base_filename = os.path.splitext(filename)[0]

                # 이미지 저장
                binary_filename = os.path.join(binary_dir, f"{base_filename}.jpg")
                semantic_filename = os.path.join(semantic_dir, f"{base_filename}.jpg")

                cv2.imwrite(binary_filename, binary_result)
                cv2.imwrite(semantic_filename, semantic_result)

            QMessageBox.information(self, "저장", "성공적으로 저장되었습니다.")
            save_label_statistics(
                self.image_labels, os.path.join(current_dir, "label_stats")
            )
            self.set_list()
        except Exception as e:
            print(f"savepoint 오류: {e}")
            QMessageBox.warning(self, "오류", f"포인트 저장 실패: {e}")

    def load_labels_from_file(self):
        """
        이전에 생성한 binary_cd 또는 semantic_cd 이미지에서 폴리곤을 검출하는 기능
        현재 버전에서는 CSV 저장을 제거했으므로 이 함수는 초기 상태만 설정합니다
        """
        try:
            # 새 이미지에 대한 빈 폴리곤 리스트 초기화
            self.before_box.poly_list = []
            self.after_box.poly_list = self.before_box.poly_list  # 페어 이미지와 공유
            self.image_labels[self.before_box.path] = self.before_box.poly_list

        except Exception as e:
            print(f"라벨 초기화 오류: {e}")
            QMessageBox.warning(self, "오류", f"라벨 초기화 중 오류가 발생했습니다: {e}")

    def set_list(self):
        """
        리스트 업데이트 메서드, 선택된 클래스와 폴리곤 좌표 표시
        """
        try:
            # 폴리곤 정보 업데이트
            poly_info = []
            for poly_dict in self.before_box.poly_list:
                class_number = poly_dict['class']
                points = poly_dict['points']

                # 클래스 이름 변환
                class_name = {
                    1: "사람",
                    2: "돌",
                    3: "흙",
                    4: "물",
                    5: "불",
                    6: "나무"
                }.get(class_number, f"알 수 없음({class_number})")

                poly_info.append(f"{class_name}: {points}")

            # Base Image와 Temporary B 리스트 굵게 표시 적용
            self.LV_A.setModel(self.create_bold_model(self.temp_listA, self.selected_a_image_path))
            self.LV_B.setModel(self.create_bold_model(self.temp_listB, self.selected_b_image_path))
            self.LV_label.setModel(QStringListModel(poly_info))

        except Exception as e:
            print(f"set_list 오류: {e}")

    def create_bold_model(self, file_list, selected_file):
        """
        리스트에서 선택된 파일을 굵게 처리하는 모델 생성
        """
        model = QStandardItemModel()
        for file in file_list:
            item = QStandardItem(os.path.basename(file))  # 파일 경로 대신 파일명만 표시
            if file == selected_file:
                # 굵게 표시
                font = item.font()
                font.setBold(True)
                item.setFont(font)
            model.appendRow(item)
        return model

    def closeEvent(self, event):
        reply = QMessageBox.question(self, '종료', '정말 종료하시겠습니까?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            if self.auto_adding_points:
                self.auto_add_timer.stop()
            event.accept()
        else:
            event.ignore()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = change_detection()
    win.show()
    win.setFocus()
    sys.exit(app.exec_())
