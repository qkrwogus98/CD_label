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
    """
    이미지 라벨 통계를 저장하는 함수.
    각 클래스별로 해당 클래스가 포함된 이미지 파일명 목록을 txt 파일로 저장합니다.
    """
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
        txt_path = os.path.join(output_dir, f"{label}.txt")
        existing_files = set()

        # 기존 파일이 있다면 읽기
        if os.path.exists(txt_path):
            with open(txt_path, "r", encoding="utf-8") as f:
                existing_files = set(line.strip() for line in f.readlines())

        # 새 파일 목록 추가
        all_files = existing_files.union(files)

        # 다시 저장 (정렬된 상태로)
        with open(txt_path, "w", encoding="utf-8") as f:
            f.write("\n".join(sorted(all_files, key=natural_key)))


def append_no_change_file(output_dir, selected_a_image_path, selected_b_image_path):
    """
    '변화없음.txt' 파일에 변화가 없는 이미지 파일명을 추가하는 함수.
    before와 after 이미지 파일명이 동일한 경우에만 저장하고, 중복 저장을 방지합니다.
    """
    os.makedirs(output_dir, exist_ok=True)
    no_change_file = os.path.join(output_dir, "변화없음.txt")

    filename = os.path.basename(selected_a_image_path)

    # 파일명이 같은지 확인
    if filename != os.path.basename(selected_b_image_path):
        print("[경고] before와 after 파일명이 다릅니다. 저장 안 함.")
        return

    # 중복 저장 방지
    if os.path.exists(no_change_file):
        with open(no_change_file, "r", encoding="utf-8") as f:
            existing = set(line.strip() for line in f.readlines())
            if filename in existing:
                return

    # 한 줄만 저장
    with open(no_change_file, "a", encoding="utf-8") as f:
        f.write(filename + "\n")


# 전역 예외 처리기
def exception_hook(exctype, value, tb):
    """
    처리되지 않은 예외 발생 시 호출되어 오류 메시지를 표시하는 전역 예외 처리기.
    """
    tb_msg = ''.join(traceback.format_exception(exctype, value, tb))
    print(tb_msg) # 콘솔에도 오류 출력
    QMessageBox.critical(None, "오류 발생", f"프로그램 실행 중 오류가 발생했습니다:\n\n{tb_msg}\n\n자세한 내용은 콘솔 출력을 확인해주세요.")
    # 프로그램 종료하지 않음


sys.excepthook = exception_hook

class ImageBox(QWidget):
    """
    이미지를 표시하고 사용자와 상호작용하는 위젯 (폴리곤 그리기, 이미지 이동/확대/축소 등).
    변화 전(before)과 변화 후(after) 이미지를 각각 담당합니다.
    """
    def __init__(self, is_after=False):
        super(ImageBox, self).__init__()

        # 이미지가 변화 후 이미지인지 표시하는 플래그
        self.is_after = is_after

        # 라벨 리스트 (두 이미지 간 공유될 수 있음, 실제로는 bigbox의 image_labels를 통해 관리됨)
        self.poly_list = [] # 현재 이미지에 그려진 폴리곤 목록

        # 이미지 관련 변수들
        self.path = None  # 원본 이미지 경로
        self.scale = 1.0 # 이미지 확대/축소 비율
        self.w = None # 화면에 표시될 이미지 너비
        self.h = None # 화면에 표시될 이미지 높이
        self.point = QPoint(0, 0) # 이미지가 위젯 내에 그려질 시작점 (이동 시 변경됨)

        # 원본 이미지 (QPixmap)
        self.img = None
        # self.pair_img = None  # 페어 이미지 (변화 전/후) - 현재 코드에서는 직접 사용되지 않음

        # 상태 플래그들
        self.start_pos = None # 마우스 클릭 시작 위치 (이미지 이동용)
        self.end_pos = None # 마우스 이동 종료 위치 (이미지 이동용)
        self.is_left_clicked = False # 마우스 왼쪽 버튼 클릭 여부
        self.is_moving = False # 이미지 이동 중인지 여부
        self.setCursor(Qt.PointingHandCursor) # 기본 커서 모양
        self.is_drawing = False # 폴리곤 그리기 중인지 여부
        self.line = [] # 현재 그리고 있는 폴리곤의 점들 (상대 좌표)
        self.pos = None # 현재 마우스 커서 위치 (그리기 중일 때)
        self.is_closed = False # 폴리곤이 닫힐 수 있는 상태인지 (시작점 근처에 마우스가 있을 때)

        # 추가 변수들
        self.current_class = 1  # 기본 클래스 번호 (1: 사람)
        self.selected_poly_index = -1  # 선택된 폴리곤 인덱스 (리스트뷰에서 선택 시 강조용)

        # 키보드 이벤트 수신 가능하도록 설정
        self.setFocusPolicy(Qt.ClickFocus)

        # 라인 언두/리두 스택 (폴리곤 그리기 중 점 취소/복구용)
        self.line_redo_stack = []

        # 페어 이미지 박스 참조 (동기화용)
        self.pair_box = None
        self.bigbox = None # 부모 위젯 (change_detection 인스턴스) 참조

    def set_pair_box(self, box):
        """페어 이미지 박스 설정 (동기화용)"""
        self.pair_box = box

    def handle_polygon_class_input(self, pos):
        """
        주어진 위치(pos)가 포함된 폴리곤을 찾아 클래스를 입력받는 대화상자를 표시하는 메서드.
        (현재 코드에서는 직접 호출되지 않고, mouseDoubleClickEvent에서 유사 로직 사용)
        """
        try:
            for index, poly_dict in enumerate(self.poly_list):
                points = poly_dict['points']
                # points의 길이가 유효한지 확인 (최소 2개의 점, 즉 4개의 좌표값)
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
                        value=poly_dict.get('class', self.current_class), min=1, max=6
                    )

                    if ok:
                        # 선택된 폴리곤의 클래스 정보를 업데이트
                        self.poly_list[index]['class'] = class_number

                        # 이미지 라벨 업데이트 (부모 위젯의 image_labels 딕셔너리 업데이트)
                        if self.bigbox and self.path:
                            self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        self.repaint()
                        # 페어 이미지도 업데이트
                        if self.pair_box:
                            self.pair_box.repaint()
                    break  # 탐지된 폴리곤이 처리되었으므로 루프 종료
        except Exception as e:
            print(f"handle_polygon_class_input 오류: {e}")
            QMessageBox.warning(self, "오류", f"폴리곤 클래스 입력 중 오류 발생: {e}")


    def mouseDoubleClickEvent(self, e):
        """
        마우스 더블 클릭 이벤트 처리.
        클릭한 위치에 폴리곤이 있으면 해당 폴리곤의 클래스를 수정하는 다이얼로그를 띄웁니다.
        """
        try:
            # 자동 폴리곤 모드에서는 더블클릭 다르게 처리하거나 비활성화 가능
            if self.bigbox and self.bigbox.auto_polygon_mode:
                return # 자동 폴리곤 모드에서는 기존 더블클릭 동작 안 함

            click_pos = e.pos() # 클릭된 위젯 내 좌표
            for index, poly_dict in enumerate(self.poly_list):
                points = poly_dict['points']
                # points의 길이가 유효한지 확인
                if len(points) < 4 or len(points) % 2 != 0:
                    # print(f"폴리곤 {index}의 points 길이가 올바르지 않습니다: {len(points)}")
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
                                                           value=poly_dict.get('class', self.current_class), min=1, max=6)
                    if ok:
                        # 현재 상태를 undo 스택에 저장
                        if self.bigbox:
                            self.bigbox.push_undo()
                        # 선택된 폴리곤의 클래스 업데이트
                        self.poly_list[index]['class'] = class_number
                        # 이미지 라벨 업데이트 (부모 위젯의 image_labels 딕셔너리 업데이트)
                        if self.bigbox and self.path:
                            self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        self.repaint()
                        # 페어 이미지도 업데이트 (poly_list는 공유되므로 repaint만 호출)
                        if self.pair_box:
                            self.pair_box.repaint()
                        # 리스트 뷰 업데이트
                        if self.bigbox:
                            self.bigbox.set_list_models()
                    break # 하나의 폴리곤만 처리
        except Exception as e:
            print(f"mouseDoubleClickEvent 오류: {e}")
            QMessageBox.warning(self, "오류", f"더블 클릭 이벤트 처리 중 오류 발생: {e}")

    def set_image(self):
        """
        이미지를 설정하고 위젯 크기를 이미지 크기에 맞게 조정.
        한글 경로 문제를 처리하기 위해 cv2.imdecode 사용.
        """
        try:
            if not self.path or not os.path.exists(self.path):
                # print(f"이미지 경로가 유효하지 않거나 파일이 존재하지 않습니다: {self.path}")
                self.img = None # 이미지를 None으로 설정하여 빈 화면 표시
                self.setFixedSize(400,400) # 기본 크기 설정
                self.repaint()
                return

            img_array = np.fromfile(self.path, np.uint8)  # 한글 경로 문제로 우회
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)  # 이미지 디코딩
            if img is None:
                raise ValueError(f"cv2.imdecode가 이미지를 로드하지 못했습니다: {self.path}")

            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # RGB 순서로 변환
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qimg = QImage(img.data, width, height, bytesPerLine, QImage.Format_RGB888)
            self.img = QPixmap.fromImage(qimg)


            # 변화 감지 모드가 활성화되어 있었다면 플래그 설정 (원본 이미지 로드 시 비활성화)
            if (
                hasattr(self.bigbox, "change_detection_active")
                and self.bigbox.change_detection_active
            ):
                # 이 부분은 detect_pixel_changes에서 이미지를 직접 설정하므로,
                # 일반 set_image 호출 시에는 change_detection_active를 False로 만들 필요는 없음.
                # reset_images 함수에서 처리하도록 둠.
                pass


        except Exception as e:
            print(f"이미지 로드 오류 ({self.path}): {e}")
            # 오류 발생 시 빈 QPixmap 설정 또는 기본 이미지 표시
            self.img = QPixmap() # 빈 QPixmap
            QMessageBox.warning(self, "이미지 로드 오류", f"이미지를 불러오는 데 실패했습니다: {self.path}\n{e}")


        try:
            # 스케일 값 초기화
            self.scale = 1.0
            # 이미지 크기 초기화
            if self.img and not self.img.isNull():
                self.w = self.img.width()
                self.h = self.img.height()
                # 위젯 크기 설정 - 이미지의 실제 크기로 설정
                self.setMinimumSize(100, 100) # 최소 크기 보장
                self.setFixedSize(self.w, self.h)
            else: # 이미지가 없거나 로드 실패 시
                self.w = 400 # 기본 너비
                self.h = 400 # 기본 높이
                self.setFixedSize(self.w, self.h)


            self.point = QPoint(0, 0) # 이미지 표시 위치 초기화
            self.repaint()
        except Exception as e:
            print(f"set_image 크기 설정 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 설정 실패: {e}")


    def paintEvent(self, e):
        """
        페인트 이벤트 수신: 이미지, 폴리곤, 미완성 선, 상태 텍스트를 모두 그리기
        """
        try:
            painter = QPainter()
            painter.begin(self)

            if self.img and not self.img.isNull():
                # 이미지 그리기
                # target rect와 source rect를 사용하여 QPixmap의 일부를 그릴 수 있음
                # 여기서는 전체 이미지를 self.point 위치에 self.w, self.h 크기로 그림
                target_rect = QPoint(int(self.point.x()), int(self.point.y()))
                # painter.drawPixmap(target_rect, self.img, self.img.rect()) # 원본 크기로만 그림
                # 스케일된 크기로 그리려면 drawPixmap(target_x, target_y, scaled_w, scaled_h, source_pixmap)
                painter.drawPixmap(int(self.point.x()), int(self.point.y()), int(self.w), int(self.h), self.img)


                # 완성된 폴리곤 그리기
                for index, poly_dict in enumerate(self.poly_list):
                    points = poly_dict.get('points', []) # points가 없을 경우 대비
                    class_number = poly_dict.get('class', self.current_class) # class가 없을 경우 대비

                    # 클래스 번호에 따른 색상 설정
                    pen_color = self.get_class_color(class_number)
                    pen = QPen(pen_color)
                    pen.setWidth(3)

                    # 선택된 폴리곤을 두꺼운 선으로 강조 표시
                    if index == self.selected_poly_index:
                        pen.setWidth(5)  # 선택된 폴리곤 강조

                    painter.setPen(pen)
                    brush_color = self.get_class_color(class_number, alpha=50) # 반투명 채우기
                    brush = QBrush(brush_color)
                    painter.setBrush(brush)

                    # points의 길이가 유효한지 확인
                    if len(points) < 4 or len(points) % 2 != 0:
                        # print(f"폴리곤 {index}의 points 길이가 올바르지 않습니다: {len(points)}")
                        continue

                    # 폴리곤 좌표를 QPolygon으로 변환 (절대 좌표 사용)
                    poly_qpolygon = QPolygon()
                    for i in range(0, len(points), 2):
                        # get_absolute_coor은 리스트를 반환하므로, 첫 번째 점의 x, y를 가져옴
                        abs_coords = self.get_absolute_coor([[points[i], points[i + 1]]])
                        if len(abs_coords) >= 2:
                            x, y = abs_coords[0], abs_coords[1]
                            if np.isnan(x) or np.isnan(y):
                                continue
                            poly_qpolygon.append(QPoint(int(x), int(y)))
                    if poly_qpolygon.size() > 1: # 점이 2개 이상일 때만 그림
                        painter.drawPolygon(poly_qpolygon)

                # 미완성된 선 그리기 (선택된 클래스 색상)
                if self.is_drawing and self.pos and self.line:
                    pen = QPen(self.get_class_color(self.current_class))
                    pen.setWidth(3)
                    painter.setPen(pen)
                    painter.setBrush(Qt.NoBrush) # 채우기 없음

                    if len(self.line) >=2:
                        start_coords = self.get_absolute_coor([[self.line[0], self.line[1]]])
                        if len(start_coords) >=2:
                            start_x, start_y = start_coords[0], start_coords[1]
                            painter.drawEllipse(QPoint(int(start_x), int(start_y)), 5, 5)  # 시작점 표시

                    # 닫힌 폴리곤을 완성할 경우 (시작점 근처에 원 표시)
                    if self.is_closed and len(self.line) >=2:
                        # is_closed는 mouseMoveEvent에서 설정됨
                        # 시작점 좌표를 다시 가져와서 그림
                        end_coords = self.get_absolute_coor([[self.line[0], self.line[1]]])
                        if len(end_coords) >=2:
                            end_x, end_y = end_coords[0], end_coords[1]
                            painter.drawEllipse(QPoint(int(end_x), int(end_y)), 10, 10) # 더 큰 원으로 표시

                    # 선 그리기
                    for i in range(0, len(self.line) - 2, 2):
                        p1_coords = self.get_absolute_coor([[self.line[i], self.line[i + 1]]])
                        p2_coords = self.get_absolute_coor([[self.line[i + 2], self.line[i + 3]]])

                        if len(p1_coords) >=2 and len(p2_coords) >=2:
                            x1, y1 = p1_coords[0], p1_coords[1]
                            x2, y2 = p2_coords[0], p2_coords[1]
                            if np.isnan(x1) or np.isnan(y1) or np.isnan(x2) or np.isnan(y2):
                                continue
                            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

                    # 마지막 선 그리기 (현재 마우스 위치까지)
                    if self.pos and len(self.line) >=2 : # self.line에 최소 한 점은 있어야 함
                        last_point_coords = self.get_absolute_coor([[self.line[-2], self.line[-1]]])
                        if len(last_point_coords) >= 2:
                            last_x, last_y = last_point_coords[0], last_point_coords[1]
                            painter.drawLine(int(last_x), int(last_y), int(self.pos.x()), int(self.pos.y()))
            else: # 이미지가 없거나 로드 실패 시
                # 회색 배경 또는 안내 메시지 표시
                painter.fillRect(self.rect(), QColor(200, 200, 200))
                painter.setPen(Qt.black)
                painter.drawText(self.rect(), Qt.AlignCenter, "이미지를 불러오세요.")


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

        except Exception as e:
            print(f"paintEvent 오류: {e}")
            # 오류 발생 시 painter가 정상적으로 end() 되도록 보장
            if painter.isActive():
                painter.end()


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
            color = QColor(0, 0, 0, alpha)  # 기본 검정색 (알 수 없는 클래스)
        return color

    def get_absolute_coor(self, coord_list):
        """
        상대적인 좌표(이미지 원본 기준)를 현재 위젯에 표시될 절대 좌표로 변환.
        coord_list: [[x1, y1], [x2, y2], ...] 형태의 리스트
        반환: [abs_x1, abs_y1, abs_x2, abs_y2, ...] 형태의 1차원 리스트
        """
        abs_list = []
        for coor in coord_list:
            if len(coor) == 2: # 좌표 쌍이 올바른지 확인
                # 이미지의 현재 표시 위치(self.point)와 스케일(self.scale)을 고려
                x1 = self.point.x() + self.scale * coor[0]
                y1 = self.point.y() + self.scale * coor[1]
                abs_list.extend([x1, y1])
        return abs_list

    def get_relative_coor(self, abs_pos_widget):
        """
        절대 좌표(위젯 기준)를 이미지 원본 기준의 상대 좌표로 변환.
        abs_pos_widget: QPoint 형태의 위젯 기준 절대 좌표
        반환: QPointF 형태의 이미지 원본 기준 상대 좌표 (소수점 포함 가능)
        """
        if self.scale == 0: # 0으로 나누기 방지
            return QPoint(0,0) # 또는 QPointF(0.0, 0.0)

        # 위젯 좌표에서 이미지 시작점(self.point)을 빼고 스케일로 나눔
        # 이것은 이미지 위젯 내에서의 상대 좌표를 의미함.
        # 만약 change_mask에 대한 상대 좌표를 원한다면, change_mask는 원본 이미지 크기이므로
        # self.point와 self.scale을 모두 고려해야 함.
        # 즉, ( (위젯X - 이미지표시시작X) / 스케일, (위젯Y - 이미지표시시작Y) / 스케일 )
        img_rel_x = (abs_pos_widget.x() - self.point.x()) / self.scale
        img_rel_y = (abs_pos_widget.y() - self.point.y()) / self.scale
        return QPoint(int(img_rel_x), int(img_rel_y)) # OpenCV는 정수 좌표를 주로 사용


    def mouseMoveEvent(self, e):
        """
        마우스 이동 이벤트 처리
        - 왼쪽 버튼 클릭 상태에서 이미지 이동
        - 폴리곤 그리기 중일 때 현재 마우스 위치 업데이트 및  repaint
        """
        try:
            # 이미지 이동 (자동 폴리곤 모드가 아닐 때 또는 그리기 중이 아닐 때)
            is_auto_poly_mode = self.bigbox and self.bigbox.auto_polygon_mode
            if self.is_left_clicked and not self.is_drawing and not is_auto_poly_mode:
                self.end_pos = e.pos() - self.start_pos # 현재 위치와 시작 위치의 차이 계산
                old_x, old_y = self.point.x(), self.point.y() # 이전 위치 저장
                self.point = self.point + self.end_pos # 이미지 표시 시작점 업데이트
                self.start_pos = e.pos() # 다음 이동을 위해 시작 위치 업데이트
                self.repaint()
                self.is_moving = True # 이동 중 플래그 설정

                # 페어 이미지 동기화
                if self.pair_box and not self.pair_box.is_left_clicked: # 상대방이 드래그 중이 아닐 때만
                    dx = self.point.x() - old_x
                    dy = self.point.y() - old_y
                    self.pair_box.point = QPoint(self.pair_box.point.x() + dx, self.pair_box.point.y() + dy)
                    self.pair_box.repaint()

            # 선 그리기 중 마우스 위치 기록
            if self.is_drawing:
                self.pos = e.pos() # 현재 마우스 위치 (절대 좌표)
                if len(self.line) >= 2: # 최소 한 개의 점이 그려진 상태
                    # 시작점 좌표 (절대 좌표)
                    start_abs_coords = self.get_absolute_coor([[self.line[0], self.line[1]]])
                    if len(start_abs_coords) >= 2:
                        x1_abs, y1_abs = start_abs_coords[0], start_abs_coords[1]
                        # 현재 마우스 위치와 시작점 사이의 거리 계산 (절대 좌표 기준)
                        # 폴리곤을 닫을 수 있는지 (시작점 근처인지) 확인
                        if abs(self.pos.x() - x1_abs) < 10 and abs(self.pos.y() - y1_abs) < 10 and len(self.line) > 4: # 최소 3개의 점이 있어야 닫을 수 있음 (좌표는 6개)
                            self.is_closed = True
                        else:
                            self.is_closed = False
                self.repaint() # 실시간으로 선이 그려지는 것을 보여주기 위해 repaint
        except Exception as e:
            print(f"mouseMoveEvent 오류: {e}")


    def mousePressEvent(self, e):
        """마우스 버튼 누름 이벤트 처리"""
        if e.button() == Qt.LeftButton:
            self.setFocus() # 키보드 이벤트 수신을 위해 포커스 설정

            # 자동 폴리곤 생성 모드 처리
            if self.bigbox and self.bigbox.auto_polygon_mode and self.bigbox.change_mask is not None:
                # 변화 감지 마스크가 있는 경우에만 자동 생성 시도
                self.try_auto_create_polygon(e.pos())
                # 자동 폴리곤 모드에서는 일반적인 점 그리기나 이미지 이동을 막을 수 있음
                # 여기서는 try_auto_create_polygon이 성공하면 is_drawing 등을 설정하지 않도록 함
                return # 자동 폴리곤 생성 시도 후 종료

            # 일반 모드 또는 자동 폴리곤 조건 미충족 시 기존 로직 수행
            self.is_left_clicked = True
            self.start_pos = e.pos() # 클릭 시작 위치 저장 (이미지 이동 또는 점 추가용)
            self.is_moving = False # 클릭 시작 시에는 이동 중이 아님

    def mouseReleaseEvent(self, e):
        """마우스 버튼 떼어짐 이벤트 처리"""
        try:
            # 자동 폴리곤 모드에서는 Press에서 처리했으므로 Release에서 특별히 할 일 없음
            if self.bigbox and self.bigbox.auto_polygon_mode and self.bigbox.change_mask is not None and e.button() == Qt.LeftButton:
                return

            if e.button() == Qt.LeftButton:
                self.is_left_clicked = False
                # 선 또는 점 기록 (이미지 이동 중이 아니었을 때만)
                if not self.is_moving:
                    # 클릭된 절대 위치(e.pos())를 상대 좌표로 변환하여 저장
                    relative_pos = self.get_relative_coor(e.pos())

                    # 선 그리기 시작 또는 점 추가
                    if not self.is_drawing: # 첫 클릭 시 그리기 시작
                        self.is_drawing = True
                        self.line = [] # 새 라인 시작
                        self.line_redo_stack.clear() # 새 라인 시작 시 redo 스택 비우기

                    self.update_line(relative_pos) # 점 추가
                self.is_moving = False # 이동 종료

            if e.button() == Qt.RightButton and not self.is_moving and self.is_drawing:
                # 오른쪽 클릭 시 그리기 완료/취소 메뉴
                rightMenu = QMenu(self)
                finish_act = QAction("완료", self,
                                     triggered=lambda: self.update_line(None, "finish"))
                cancel_act = QAction("취소", self,
                                     triggered=lambda: self.update_line(None, "cancel"))
                rightMenu.addAction(finish_act)
                rightMenu.addAction(cancel_act)
                rightMenu.exec_(QCursor.pos()) # 현재 마우스 커서 위치에 메뉴 표시
        except Exception as e:
            print(f"mouseReleaseEvent 오류: {e}")
            QMessageBox.warning(self, "오류", f"마우스 릴리즈 이벤트 처리 중 오류 발생: {e}")

    def try_auto_create_polygon(self, click_pos_widget):
        """
        자동 폴리곤 모드에서 사용자가 클릭한 위치를 기반으로 폴리곤을 자동 생성 시도.
        click_pos_widget: QPoint, 위젯 내 클릭된 절대 좌표.
        """
        print(f"자동 폴리곤 생성 시도: {click_pos_widget}")
        if not self.bigbox or not self.bigbox.change_mask is not None:
            print("자동 폴리곤 생성 실패: 변화 감지 마스크 없음.")
            return

        # 1. 클릭된 위젯 좌표를 원본 이미지 (change_mask) 기준으로 변환
        #    get_relative_coor는 이미지 표시 영역 기준이므로, change_mask는 원본 이미지 전체 기준.
        #    change_mask는 스케일링되지 않은 원본 이미지 크기.
        #    클릭 좌표 (위젯 기준) -> 이미지 표시 영역 내 상대 좌표 -> 원본 이미지 기준 좌표
        #    self.point: 이미지 표시 시작점 (위젯 기준)
        #    self.scale: 현재 이미지 확대/축소 비율
        #    change_mask_x = (click_pos_widget.x() - self.point.x()) / self.scale
        #    change_mask_y = (click_pos_widget.y() - self.point.y()) / self.scale
        #    click_pt_on_mask = (int(change_mask_x), int(change_mask_y))
        click_pt_on_mask_qpoint = self.get_relative_coor(click_pos_widget) # 정수형 QPoint 반환
        click_pt_on_mask = (click_pt_on_mask_qpoint.x(), click_pt_on_mask_qpoint.y())


        # change_mask의 크기 확인
        mask_h, mask_w = self.bigbox.change_mask.shape[:2]
        if not (0 <= click_pt_on_mask[0] < mask_w and 0 <= click_pt_on_mask[1] < mask_h):
            print("클릭 위치가 마스크 범위를 벗어났습니다.")
            return

        # 2. 클릭된 위치가 change_mask에서 흰색(변화 감지된 영역)인지 확인
        if self.bigbox.change_mask[click_pt_on_mask[1], click_pt_on_mask[0]] == 0: # 검은색이면
            print("클릭 위치는 변화가 감지된 영역이 아닙니다.")
            # 또는 가장 가까운 흰색 영역을 찾는 로직 추가 가능
            return

        # 3. OpenCV를 사용하여 change_mask에서 모든 contours 찾기
        #    cv2.findContours는 원본 이미지를 변경하므로 복사본 사용
        contours, hierarchy = cv2.findContours(self.bigbox.change_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #    RETR_EXTERNAL: 가장 바깥쪽 외곽선만 찾음
        #    CHAIN_APPROX_SIMPLE: 폴리곤 꼭지점 수를 줄여줌

        if not contours:
            print("마스크에서 contour를 찾지 못했습니다.")
            return

        # 4. 클릭된 점을 포함하는 contour 찾기
        selected_contour = None
        for contour in contours:
            # cv2.pointPolygonTest: 점이 contour 내부에 있으면 양수, 외부에 음수, 경계에 0 반환
            # 세 번째 인자 True: 내부/외부/경계까지의 거리 반환, False: +1, -1, 0 중 하나 반환
            if cv2.pointPolygonTest(contour, (click_pt_on_mask[0], click_pt_on_mask[1]), False) >= 0:
                selected_contour = contour
                break

        if selected_contour is None:
            print("클릭 위치를 포함하는 contour를 찾지 못했습니다.")
            return

        # 5. (선택 사항) contour 단순화 (cv2.approxPolyDP)
        #    epsilon 값은 contour 둘레의 일정 비율로 설정 가능
        #    epsilon = 0.01 * cv2.arcLength(selected_contour, True)
        #    approx_contour = cv2.approxPolyDP(selected_contour, epsilon, True)
        #    polygon_points_cv = approx_contour.reshape(-1, 2) # (N, 1, 2) -> (N, 2) 형태로 변경
        polygon_points_cv = selected_contour.reshape(-1, 2) # 단순화 없이 사용

        # 6. contour 점들을 self.line 형식 (상대 좌표 [x1, y1, x2, y2...])으로 변환
        #    contour 점들은 원본 이미지 기준이므로, ImageBox의 line도 원본 이미지 기준 상대 좌표.
        new_line = []
        for pt in polygon_points_cv:
            new_line.append(float(pt[0])) # x 좌표
            new_line.append(float(pt[1])) # y 좌표

        if len(new_line) < 6: # 최소 3개의 점 필요
            print("자동 생성된 폴리곤의 점 개수가 너무 적습니다.")
            return

        # 7. 현재 line, poly_list 등에 추가하고 repaint
        #    기존 그리기 로직과 유사하게 처리
        if self.bigbox:
            self.bigbox.push_undo() # 작업 전 상태 저장

        self.line = new_line # 자동 생성된 라인으로 설정
        # 바로 폴리곤으로 확정
        self.poly_list.append({'points': self.line.copy(), 'class': self.current_class})
        self.line = [] # 현재 라인 초기화

        if self.bigbox and self.path:
            self.bigbox.image_labels[self.path] = self.poly_list.copy()
            self.bigbox.redo_stack.clear() # 새 작업이므로 redo 스택 비우기
            self.bigbox.set_list_models()

        self.is_drawing = False # 그리기가 완료된 것으로 간주
        self.is_closed = False
        self.pos = None
        self.repaint()

        if self.pair_box:
            self.pair_box.repaint() # 페어 이미지도 업데이트

        print("자동 폴리곤 생성 완료!")


    def update_line(self, rel_pos=None, flag="draw"):
        """
        폴리곤 선 업데이트 로직.
        rel_pos: 상대 좌표 (QPoint)
        flag: "draw", "finish", "cancel"
        """
        try:
            if flag == "cancel":
                self.line = []
                self.line_redo_stack.clear()
                self.is_drawing = False
                self.is_closed = False
                self.pos = None # 현재 마우스 위치 추적 중단
                self.repaint()
            elif flag == "finish":
                if len(self.line) > 4: # 최소 3개의 점 (6개의 좌표값)이 있어야 폴리곤 완성
                    # 현재 상태를 undo 스택에 저장 (메인 윈도우의 undo 스택 사용)
                    if self.bigbox:
                        self.bigbox.push_undo()

                    self.is_drawing = False
                    self.is_closed = False
                    self.pos = None
                    # 클래스 정보와 함께 폴리곤 저장
                    self.poly_list.append({'points': self.line.copy(), 'class': self.current_class})
                    self.line = [] # 현재 라인 초기화
                    self.line_redo_stack.clear() # 리두 스택 초기화

                    # 이미지 라벨 업데이트 (부모 위젯의 image_labels 딕셔너리 업데이트)
                    if self.bigbox and self.path:
                        self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        # redo 스택 비우기 (새로운 작업이므로)
                        self.bigbox.redo_stack.clear()
                        self.bigbox.set_list_models() # 라벨 리스트 뷰 업데이트

                    # 페어 이미지도 업데이트 (poly_list는 공유되므로 repaint만 호출)
                    if self.pair_box and self.bigbox:
                        self.pair_box.repaint()


                    # 자동 포인트 추가 기능 해제 (필요 시)
                    if self.bigbox and self.bigbox.auto_adding_points:
                        self.bigbox.auto_adding_points = False
                        self.bigbox.auto_add_timer.stop()
                else:
                    # 점이 두 개 이하일 때 취소 (폴리곤을 만들 수 없음)
                    self.update_line(None, "cancel")
                self.repaint()
            elif flag == "draw" and rel_pos is not None: # 점 추가
                if self.is_closed: # 시작점 근처에서 클릭하면 폴리곤 완성
                    self.update_line(None, "finish")
                else:
                    # 점 업데이트 (상대 좌표로 저장)
                    self.line.append(rel_pos.x())
                    self.line.append(rel_pos.y())
                    # 점 추가 시 redo 스택 초기화
                    self.line_redo_stack.clear()
                    self.repaint()
        except Exception as e:
            print(f"update_line 오류: {e}")
            QMessageBox.warning(self, "오류", f"선 업데이트 실패: {e}")


    def keyPressEvent(self, event):
        """
        키보드 이벤트 처리
        - A: 현재 마우스 위치에 점 추가 (왼쪽 클릭 시뮬레이션)
        - Ctrl+Z: 그리기 중일 때 마지막 점 취소 / 그리기 완료 후 전체 폴리곤 취소 (undo)
        - Ctrl+Y: 그리기 중일 때 취소한 점 복구 / 전체 폴리곤 복구 (redo)
        """
        if event.key() == Qt.Key_A:
            # 자동 폴리곤 모드에서는 A키 동작 안 함
            if self.bigbox and self.bigbox.auto_polygon_mode:
                return

            # 현재 마우스 위치에 왼쪽 클릭 이벤트 시뮬레이션 (점 추가)
            if not self.is_drawing: # 첫 A키 입력 시 그리기 시작
                self.is_drawing = True
                self.line = []
                self.line_redo_stack.clear()

            pos = self.mapFromGlobal(QCursor.pos()) # 위젯 내부 좌표로 변환
            self.is_moving = False # 이동 중이 아님을 명시
            # QMouseEvent 생성 시 버튼 상태와 수정자 키도 정확히 설정하는 것이 좋음
            synthetic_event = QMouseEvent(QEvent.MouseButtonRelease, pos, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
            self.mouseReleaseEvent(synthetic_event) # mouseReleaseEvent 로직 재활용

        elif event.key() == Qt.Key_Z and event.modifiers() & Qt.ControlModifier: # Ctrl+Z
            if self.is_drawing: # 폴리곤 그리기 중 (점 취소)
                if self.line and len(self.line) >= 2:
                    # 마지막 점 (x, y 좌표)을 redo 스택에 추가
                    self.line_redo_stack.append(self.line[-2:])
                    self.line = self.line[:-2] # 마지막 점 제거
                    self.is_closed = False # 점이 제거되었으므로 닫힘 상태 해제
                    self.pos = self.mapFromGlobal(QCursor.pos()) # 현재 마우스 위치 업데이트
                    self.repaint()
            elif self.bigbox: # 폴리곤 그리기가 끝난 상태 (전체 작업 취소)
                self.bigbox.undo()

        elif event.key() == Qt.Key_Y and event.modifiers() & Qt.ControlModifier: # Ctrl+Y
            if self.is_drawing: # 폴리곤 그리기 중 (취소된 점 복구)
                if self.line_redo_stack:
                    self.line.extend(self.line_redo_stack.pop()) # redo 스택에서 점 복구
                    self.is_closed = False
                    self.pos = self.mapFromGlobal(QCursor.pos())
                    self.repaint()
            elif self.bigbox: # 폴리곤 그리기가 끝난 상태 (전체 작업 복구)
                self.bigbox.redo()
        else:
            super().keyPressEvent(event) # 다른 키 이벤트는 부모 클래스에서 처리


    def wheelEvent(self, event):
        """
        마우스 휠 이벤트 처리 - 이미지 확대/축소.
        마우스 커서 위치를 중심으로 확대/축소합니다.
        페어 이미지도 동기화합니다.
        """
        try:
            if not self.img or self.img.isNull(): return # 이미지가 없으면 확대/축소 안 함

            angle = event.angleDelta() / 8 # 휠 회전 각도 (보통 15도 단위)
            angleY = angle.y() # 수직 휠 값
            zoom_factor = 1.1 # 확대/축소 비율
            old_scale = self.scale

            if angleY > 0: # 휠을 위로 (줌 인)
                self.scale *= zoom_factor
            else: # 휠을 아래로 (줌 아웃)
                self.scale /= zoom_factor

            # 최대/최소 스케일 제한 (선택 사항)
            self.scale = max(0.1, min(self.scale, 10.0))


            # 이미지 크기 업데이트 (화면에 표시될 크기)
            new_w = self.img.width() * self.scale
            new_h = self.img.height() * self.scale


            # 마우스 포인터 위치를 중심으로 확대/축소
            cursor_pos_widget = event.pos() # 위젯 내 마우스 커서 위치

            # 커서 위치에 해당하는 이미지 상의 상대 좌표
            img_x_at_cursor = (cursor_pos_widget.x() - self.point.x()) / old_scale
            img_y_at_cursor = (cursor_pos_widget.y() - self.point.y()) / old_scale

            # 새 스케일에서 이미지 표시 시작점(self.point) 업데이트
            new_point_x = cursor_pos_widget.x() - (img_x_at_cursor * self.scale)
            new_point_y = cursor_pos_widget.y() - (img_y_at_cursor * self.scale)

            self.point = QPoint(int(new_point_x), int(new_point_y))
            self.w = int(new_w)
            self.h = int(new_h)


            # 위젯 크기 조정 (ScrollArea 내부 위젯이므로 FixedSize 사용)
            self.setFixedSize(self.w, self.h)
            self.repaint()

            # 페어 이미지 동기화
            if self.pair_box and self.pair_box.img and not self.pair_box.img.isNull():
                self.pair_box.scale = self.scale
                pair_new_w = self.pair_box.img.width() * self.pair_box.scale
                pair_new_h = self.pair_box.img.height() * self.pair_box.scale

                self.pair_box.point = QPoint(int(new_point_x), int(new_point_y)) # 동일한 point 사용
                self.pair_box.w = int(pair_new_w)
                self.pair_box.h = int(pair_new_h)

                self.pair_box.setFixedSize(self.pair_box.w, self.pair_box.h)
                self.pair_box.repaint()

        except Exception as e:
            print(f"wheelEvent 오류: {e}")


class SynchronizedScrollArea(QScrollArea):
    """
    두 개의 스크롤 영역 간의 스크롤 위치를 동기화하는 QScrollArea 확장 클래스.
    Shift + 마우스 휠로 내부 ImageBox의 확대/축소를 지원합니다.
    """
    def __init__(self, parent=None):
        super(SynchronizedScrollArea, self).__init__(parent)
        self.pair_scroll = None # 동기화할 상대 스크롤 영역
        self.isScrolling = False # 재귀적 동기화를 방지하기 위한 플래그

        # 스크롤바 항상 표시 (선택 사항)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

        # 스크롤 영역 속성 설정
        self.setAlignment(Qt.AlignCenter) # 내부 위젯 중앙 정렬
        self.setWidgetResizable(False)  # 내부 위젯(ImageBox)의 크기는 ImageBox 자체에서 관리

    def set_pair_scroll(self, scroll_area):
        """페어 스croll 영역 설정"""
        self.pair_scroll = scroll_area

    def scrollContentsBy(self, dx, dy):
        """스크롤 발생 시 호출되는 메서드, 스크롤 동기화 수행"""
        super().scrollContentsBy(dx, dy) # 먼저 자신의 스크롤 처리
        if self.pair_scroll and not self.isScrolling: # 상대 스크롤이 있고, 현재 동기화 중이 아닐 때
            self.pair_scroll.isScrolling = True # 상대 스크롤에서 재귀 방지

            h_value = self.horizontalScrollBar().value()
            v_value = self.verticalScrollBar().value()

            self.pair_scroll.horizontalScrollBar().setValue(h_value)
            self.pair_scroll.verticalScrollBar().setValue(v_value)

            self.pair_scroll.isScrolling = False # 동기화 완료

    def wheelEvent(self, event):
        """
        휠 이벤트 처리.
        - Shift 키와 함께 휠 사용 시 내부 ImageBox의 확대/축소 (wheelEvent 전달).
        - 일반 휠 사용 시 스크롤 및 페어 스크롤 동기화.
        """
        widget = self.widget() # 현재 스크롤 영역에 포함된 위젯 (ImageBox)
        modifiers = QApplication.keyboardModifiers()

        if widget and isinstance(widget, ImageBox) and (modifiers & Qt.ShiftModifier):
            # Shift + 휠: ImageBox의 확대/축소 처리
            widget.wheelEvent(event) # ImageBox의 wheelEvent 직접 호출
            event.accept() # 이벤트 처리 완료 알림
            return

        # 일반 휠: 스크롤 처리
        super().wheelEvent(event) # 부모 클래스의 휠 이벤트 처리 (스크롤)

        # 페어 스크롤 동기화 (scrollContentsBy와 유사한 로직)
        if self.pair_scroll and not self.isScrolling:
            self.pair_scroll.isScrolling = True

            h_value = self.horizontalScrollBar().value()
            v_value = self.verticalScrollBar().value()
            self.pair_scroll.horizontalScrollBar().setValue(h_value)
            self.pair_scroll.verticalScrollBar().setValue(v_value)

            self.pair_scroll.isScrolling = False
        event.accept()


class change_detection(QMainWindow):
    """
    메인 애플리케이션 윈도우.
    두 개의 이미지를 비교하고 변화된 부분을 라벨링하는 기능을 제공합니다.
    """
    def __init__(self, parent=None):
        super(change_detection, self).__init__(parent)
        self.temp_listA = [] # 변화 전(Before) 이미지 파일 경로 리스트
        self.temp_listB = [] # 변화 후(After) 이미지 파일 경로 리스트
        self.image_labels = {}  # 이미지 경로를 키로, 폴리곤 리스트를 값으로 저장하는 딕셔너리

        # Undo 및 Redo 스택 (폴리곤 작업 전체에 대한 실행 취소/다시 실행)
        self.undo_stack = []
        self.redo_stack = []

        # 선택된 이미지 경로를 저장하는 변수 추가
        self.selected_a_image_path = None  # 현재 선택된 Before 이미지 경로
        self.selected_b_image_path = None  # 현재 선택된 After 이미지 경로

        # 픽셀 변화 감지 관련 변수 추가
        self.change_detection_active = False  # 픽셀 변화 감지 모드 활성화 여부
        self.change_mask = None  # 변화 감지 마스크 (cv2 이미지, 흑백)
        self.change_threshold = 30  # 변화 감지 임계값 (기본값 30)
        self.original_before_img = None # 변화 감지 전의 before 이미지 (복원용)
        self.original_after_img = None  # 변화 감지 전의 after 이미지 (복원용)

        # 자동 폴리곤 생성 모드
        self.auto_polygon_mode = False


        # 윈도우 크기 설정
        self.resize(int(1400 * 0.8), int(1100 * 0.8)) # 초기 윈도우 크기

        # QTimer 설정: 단일 클릭과 더블 클릭 구분 (라벨 리스트뷰용)
        self.click_timer = QTimer()
        self.click_timer.setSingleShot(True) # 한 번만 실행
        self.click_timer.timeout.connect(self.handle_single_click_label_list) # 타임아웃 시 호출될 함수
        self.click_index_label_list = None # 클릭된 라벨 리스트의 인덱스 저장

        # 메뉴 설정
        importAct = QAction('가져오기', self, triggered=self.open_folder_dialog) # 폴더 선택으로 변경 고려
        saveAct = QAction('라벨 저장', self, triggered=self.savepoint)
        saveAct.setShortcut('Ctrl+S')
        undoAct = QAction('실행 취소 (Ctrl+Z)', self, triggered=self.undo)
        undoAct.setShortcut('Ctrl+Z') # 단축키 명시
        redoAct = QAction('다시 실행 (Ctrl+Y)', self, triggered=self.redo)
        redoAct.setShortcut('Ctrl+Y') # 단축키 명시
        exitAct = QAction('종료', self)
        exitAct.setShortcut('Ctrl+Q')
        exitAct.triggered.connect(self.close)

        bar = self.menuBar()
        file_menu = bar.addMenu("파일") # 변수명 변경 file -> file_menu
        file_menu.addActions([importAct, saveAct, undoAct, redoAct, exitAct])

        # 웹 브라우저에서 URL을 여는 QAction
        url_act = QAction("도움말 (GitHub)", self) # 텍스트 변경
        url_act.triggered.connect(lambda: webbrowser.open("https://github.com/chartgod/Changedetection_labelingtool"))
        help_menu = bar.addMenu("도움말")
        help_menu.addAction(url_act)


        # 메인 레이아웃 설정
        main_layout = QHBoxLayout()

        # 동기화 비교 뷰 설정
        self.comparison_widget = QWidget()
        comparison_layout = QHBoxLayout(self.comparison_widget)
        comparison_layout.setContentsMargins(0,0,0,0) # 여백 제거

        # 변화 전 이미지 (before)
        self.before_scroll = SynchronizedScrollArea()
        self.before_box = ImageBox(is_after=False)
        self.before_box.setMouseTracking(True) # 마우스 움직임 추적 활성화
        self.before_box.bigbox = self # 부모 참조 설정
        self.before_scroll.setWidget(self.before_box)

        # 변화 후 이미지 (after)
        self.after_scroll = SynchronizedScrollArea()
        self.after_box = ImageBox(is_after=True)
        self.after_box.setMouseTracking(True)
        self.after_box.bigbox = self
        self.after_scroll.setWidget(self.after_box)

        # 스크롤 영역 동기화 설정
        self.before_scroll.set_pair_scroll(self.after_scroll)
        self.after_scroll.set_pair_scroll(self.before_scroll)

        # 이미지 박스 동기화 설정 (poly_list 공유 및 repaint 동기화)
        self.before_box.set_pair_box(self.after_box)
        self.after_box.set_pair_box(self.before_box)
        # 중요: poly_list를 두 ImageBox가 공유하도록 설정
        self.after_box.poly_list = self.before_box.poly_list


        # 비교 뷰에 이미지 추가
        comparison_layout.addWidget(self.before_scroll)
        comparison_layout.addWidget(self.after_scroll)

        # 오른쪽 패널 위젯
        right_panel_widget = QWidget()
        V_RightPanel = QVBoxLayout(right_panel_widget)


        # 파일 리스트 영역
        file_list_layout = QHBoxLayout()
        V_A_Layout = QVBoxLayout()
        self.label_a = QLabel("변화 전(Before) 파일 목록")
        self.import_btn = QPushButton("폴더 선택...") # 버튼 텍스트 변경
        self.import_btn.clicked.connect(lambda: self.open_folder_dialog("A"))
        self.LV_A = QListView()
        self.LV_A.clicked.connect(self.on_file_list_selected)
        V_A_Layout.addWidget(self.label_a)
        V_A_Layout.addWidget(self.import_btn)
        V_A_Layout.addWidget(self.LV_A)

        V_B_Layout = QVBoxLayout()
        self.label_b = QLabel("변화 후(After) 파일 목록")
        self.import_btnB = QPushButton("폴더 선택...") # 버튼 텍스트 변경
        self.import_btnB.clicked.connect(lambda: self.open_folder_dialog("B"))
        self.LV_B = QListView()
        self.LV_B.clicked.connect(self.on_file_list_selected) # 동일한 슬롯 연결
        V_B_Layout.addWidget(self.label_b)
        V_B_Layout.addWidget(self.import_btnB)
        V_B_Layout.addWidget(self.LV_B)

        file_list_layout.addLayout(V_A_Layout)
        file_list_layout.addLayout(V_B_Layout)
        V_RightPanel.addLayout(file_list_layout)


        # 도구 영역 (저장 버튼, 클래스 선택 등)
        tool_box_widget = QWidget()
        V_Tool = QVBoxLayout(tool_box_widget)

        save_buttons_layout = QHBoxLayout()
        self.save_btn = QPushButton("라벨 저장 (Ctrl+S)")
        self.save_btn.clicked.connect(self.savepoint)
        self.no_change_save_btn = QPushButton("변화 없음으로 저장")
        self.no_change_save_btn.clicked.connect(self.save_no_change)
        save_buttons_layout.addWidget(self.save_btn)
        save_buttons_layout.addWidget(self.no_change_save_btn)
        V_Tool.addLayout(save_buttons_layout)


        # 클래스 버튼들 가로 배치
        class_buttons_group = QWidget() # 그룹핑을 위한 위젯
        H_ClassButtons = QGridLayout(class_buttons_group) # QGridLayout으로 변경하여 2줄로 배치

        self.class_buttons = {} # 버튼 관리를 위한 딕셔너리
        class_names = {1: "사람", 2: "돌", 3: "흙", 4: "물", 5: "불", 6: "나무"}
        row, col = 0, 0
        for class_num, class_name in class_names.items():
            btn = QPushButton(f"{class_name} ({class_num})")
            btn.clicked.connect(lambda checked, num=class_num: self.update_class_from_button(num))
            self.class_buttons[class_num] = btn
            H_ClassButtons.addWidget(btn, row, col)
            col += 1
            if col >= 3: # 한 줄에 3개씩
                col = 0
                row += 1
        V_Tool.addWidget(class_buttons_group)


        # 클래스 직접 입력
        class_input_layout = QHBoxLayout()
        self.class_input_label = QLabel('현재 선택 클래스:')
        self.class_input = QLineEdit()
        self.class_input.setText('1')
        self.class_input.setValidator(QIntValidator(1, 6))  # 클래스 1~6 허용
        self.class_input.returnPressed.connect(self.update_class_from_input) # 엔터 입력 시 업데이트
        self.class_input.textChanged.connect(self.highlight_class_button) # 입력값 변경 시 버튼 하이라이트
        class_input_layout.addWidget(self.class_input_label)
        class_input_layout.addWidget(self.class_input)
        V_Tool.addLayout(class_input_layout)
        self.highlight_class_button(self.class_input.text()) # 초기 버튼 하이라이트


        # 변화 감지 섹션
        change_detection_label = QLabel("픽셀 변화 감지 도구")
        change_detection_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        V_Tool.addWidget(change_detection_label)

        threshold_layout = QHBoxLayout()
        threshold_label = QLabel("감지 민감도 (낮을수록 민감):")
        self.threshold_slider = QSlider(Qt.Horizontal)
        self.threshold_slider.setMinimum(1) # 최소값 1로 변경
        self.threshold_slider.setMaximum(100)
        self.threshold_slider.setValue(self.change_threshold)
        self.threshold_slider.setToolTip("값이 낮을수록 작은 변화도 감지합니다 (1~100).")
        self.threshold_value_label = QLabel(f"{self.change_threshold}")
        self.threshold_slider.valueChanged.connect(self.update_threshold_value_label)
        threshold_layout.addWidget(threshold_label)
        threshold_layout.addWidget(self.threshold_slider)
        threshold_layout.addWidget(self.threshold_value_label)
        V_Tool.addLayout(threshold_layout)

        change_detection_btns_layout = QHBoxLayout()
        self.detect_changes_btn = QPushButton("픽셀 변화 감지 실행")
        self.detect_changes_btn.clicked.connect(self.detect_pixel_changes)
        self.reset_images_btn = QPushButton("원본 이미지로 복원")
        self.reset_images_btn.clicked.connect(self.reset_images_to_original)
        change_detection_btns_layout.addWidget(self.detect_changes_btn)
        change_detection_btns_layout.addWidget(self.reset_images_btn)
        V_Tool.addLayout(change_detection_btns_layout)

        # 자동 폴리곤 모드 버튼 추가
        self.auto_polygon_btn = QPushButton("자동 폴리곤 모드 (OFF)")
        self.auto_polygon_btn.setCheckable(True) # 토글 버튼으로 설정
        self.auto_polygon_btn.clicked.connect(self.toggle_auto_polygon_mode)
        V_Tool.addWidget(self.auto_polygon_btn)


        self.detect_changes_btn.setStyleSheet("QPushButton { background-color: #a970ff; color: white; font-weight: bold; padding: 5px; border-radius: 3px; } QPushButton:hover { background-color: #8a50e8; }")
        self.reset_images_btn.setStyleSheet("QPushButton { background-color: #555555; color: white; padding: 5px; border-radius: 3px; } QPushButton:hover { background-color: #777777; }")
        self.auto_polygon_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FF9800; } QPushButton:hover { background-color: #45a049; }")
        self.detect_changes_btn.setToolTip("두 이미지 간의 픽셀 차이를 보라색으로 표시합니다. 양쪽에 동일한 변화 후 이미지가 보여집니다.")
        self.reset_images_btn.setToolTip("변화 감지 모드를 해제하고 원래 이미지 상태로 돌아갑니다.")
        self.auto_polygon_btn.setToolTip("활성화 시, 변화 감지된 영역을 클릭하여 자동으로 폴리곤을 생성합니다.")
        self.detect_changes_btn.setMinimumHeight(30)
        self.reset_images_btn.setMinimumHeight(30)
        self.auto_polygon_btn.setMinimumHeight(30)


        V_RightPanel.addWidget(tool_box_widget) # 도구 영역 추가


        # 라벨 리스트 (폴리곤 목록)
        label_list_label = QLabel("현재 이미지 라벨 목록 (더블클릭: 수정, 우클릭: 삭제)")
        V_RightPanel.addWidget(label_list_label)
        self.LV_label = QListView()
        self.LV_label.setContextMenuPolicy(Qt.CustomContextMenu) # 우클릭 메뉴 설정
        self.LV_label.customContextMenuRequested.connect(self.show_label_list_context_menu)
        self.LV_label.clicked.connect(self.start_single_click_timer_label_list) # 단일 클릭 타이머 시작
        self.LV_label.doubleClicked.connect(self.handle_double_click_label_list) # 더블 클릭 이벤트 연결
        V_RightPanel.addWidget(self.LV_label)


        # 자동 포인트 추가 기능 (W 키 토글)
        self.auto_adding_points = False
        self.auto_add_interval = 200  # milliseconds
        self.auto_add_timer = QTimer()
        self.auto_add_timer.timeout.connect(self.add_point_at_cursor_auto)


        # 스플리터 설정
        H_Splitter = QSplitter(Qt.Horizontal)
        H_Splitter.addWidget(self.comparison_widget) # 왼쪽: 이미지 비교 뷰
        H_Splitter.addWidget(right_panel_widget)    # 오른쪽: 파일 목록, 도구, 라벨 목록
        H_Splitter.setStretchFactor(0, 3) # 왼쪽 영역이 더 넓게
        H_Splitter.setStretchFactor(1, 1)
        H_Splitter.setSizes([int(self.width() * 0.7), int(self.width() * 0.3)]) # 초기 크기 비율 설정

        main_layout.addWidget(H_Splitter)
        main_frame = QWidget()
        main_frame.setLayout(main_layout)
        self.setCentralWidget(main_frame)

        self.setWindowTitle("변화 감지 라벨링 도구 (Ver 0.2.1 - 자동 폴리곤 구조 추가)") # 버전 정보 추가
        self.setFocusPolicy(Qt.StrongFocus) # 메인 윈도우가 키보드 이벤트 우선 수신
        self.showMaximized() # 최대화된 상태로 시작
        self.set_list_models() # 초기 리스트 모델 설정

    def toggle_auto_polygon_mode(self):
        """자동 폴리곤 생성 모드를 토글합니다."""
        self.auto_polygon_mode = self.auto_polygon_btn.isChecked()
        if self.auto_polygon_mode:
            self.auto_polygon_btn.setText("자동 폴리곤 모드 (ON)")
            self.auto_polygon_btn.setStyleSheet("QPushButton:checked { background-color: #FF9800; color: white; padding: 5px; border-radius: 3px; } QPushButton { background-color: #FF9800; color: white; padding: 5px; border-radius: 3px; }") # ON 상태 스타일
            if not self.change_detection_active or self.change_mask is None:
                 QMessageBox.information(self, "알림", "자동 폴리곤 모드를 사용하려면 먼저 '픽셀 변화 감지 실행'을 해주세요.")
                 self.auto_polygon_btn.setChecked(False) # 조건 미충족 시 버튼 해제
                 self.auto_polygon_mode = False
                 self.auto_polygon_btn.setText("자동 폴리곤 모드 (OFF)")
                 self.auto_polygon_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FF9800; } QPushButton:hover { background-color: #45a049; }") # OFF 상태 스타일

        else:
            self.auto_polygon_btn.setText("자동 폴리곤 모드 (OFF)")
            self.auto_polygon_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FF9800; } QPushButton:hover { background-color: #45a049; }") # OFF 상태 스타일
        print(f"자동 폴리곤 모드: {'ON' if self.auto_polygon_mode else 'OFF'}")


    def update_threshold_value_label(self, value):
        """임계값 슬라이더 값 변경 시 라벨 업데이트 및 self.change_threshold 업데이트"""
        self.change_threshold = value
        self.threshold_value_label.setText(f"{value}")

    def detect_pixel_changes(self):
        """
        두 이미지 간의 픽셀 변화를 감지하여 보라색으로 표시합니다.
        변화 전/후 이미지 모두 변화 후 이미지를 보여주고, 변화된 부분만 보라색으로 강조합니다.
        """
        try:
            if not self.selected_a_image_path or not self.selected_b_image_path:
                QMessageBox.information(self, "알림", "변화 전/후 이미지가 모두 선택되지 않았습니다.")
                return

            # 원본 이미지 저장 (복원용) - 이미 저장되어 있지 않은 경우에만
            if self.original_before_img is None and self.before_box.img:
                self.original_before_img = self.before_box.img.copy()
            if self.original_after_img is None and self.after_box.img:
                self.original_after_img = self.after_box.img.copy()


            img_array_a = np.fromfile(self.selected_a_image_path, np.uint8)
            img_a_cv = cv2.imdecode(img_array_a, cv2.IMREAD_COLOR)

            img_array_b = np.fromfile(self.selected_b_image_path, np.uint8)
            img_b_cv = cv2.imdecode(img_array_b, cv2.IMREAD_COLOR)

            if img_a_cv is None or img_b_cv is None:
                QMessageBox.warning(self, "오류", "이미지 로드에 실패했습니다. 파일 경로를 확인해주세요.")
                return

            # 이미지 크기가 다른 경우, 작은 이미지에 맞춰 큰 이미지 리사이즈 또는 사용자에게 알림
            if img_a_cv.shape != img_b_cv.shape:
                # 여기서는 b를 a에 맞춤 (일관성을 위해)
                img_b_cv = cv2.resize(img_b_cv, (img_a_cv.shape[1], img_a_cv.shape[0]))


            # 이미지 간의 차이 계산
            diff = cv2.absdiff(img_a_cv, img_b_cv)
            # BGR 각 채널에서 임계값보다 큰 차이가 있는지 확인하여 마스크 생성
            mask = np.any(diff > self.change_threshold, axis=2).astype(np.uint8) * 255

            # 노이즈 제거를 위한 모폴로지 연산 적용 (선택 사항)
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # 작은 노이즈 제거
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # 작은 구멍 메우기

            # 생성된 마스크 저장 (이 마스크를 자동 폴리곤 생성에 사용)
            self.change_mask = mask.copy()
            print("변화 감지 마스크 (self.change_mask) 생성됨.")
            # TODO: (다음 단계) self.change_mask를 처리하여 연결된 픽셀 덩어리(contours)를
            #       미리 식별하고 저장해두면 클릭 시 더 빠르게 반응할 수 있습니다.
            #       예: self.identified_blobs = cv2.findContours(...)


            # 변화 후 이미지(img_b_cv)에 보라색으로 변화 표시
            img_b_with_changes_cv = img_b_cv.copy()
            # 보라색 (BGR 순서: 128, 0, 128) - 좀 더 연한 보라색 (180, 0, 180)도 좋음
            img_b_with_changes_cv[mask > 0] = (180, 0, 180)

            # OpenCV 이미지를 QPixmap으로 변환
            def cv_to_qpixmap(cv_img):
                rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                return QPixmap.fromImage(qt_image)

            result_pixmap = cv_to_qpixmap(img_b_with_changes_cv)

            # 양쪽 이미지 박스에 변화 감지 이미지 설정
            self.before_box.img = result_pixmap
            self.after_box.img = result_pixmap

            h, w = img_b_cv.shape[:2]
            current_scale_before = self.before_box.scale
            current_scale_after = self.after_box.scale

            self.before_box.w = int(w * current_scale_before)
            self.before_box.h = int(h * current_scale_before)
            self.before_box.setFixedSize(self.before_box.w, self.before_box.h)

            self.after_box.w = int(w * current_scale_after)
            self.after_box.h = int(h * current_scale_after)
            self.after_box.setFixedSize(self.after_box.w, self.after_box.h)


            self.change_detection_active = True # 변화 감지 모드 활성화
            self.before_box.repaint()
            self.after_box.repaint()

            QMessageBox.information(
                self,
                "픽셀 변화 감지 완료",
                f"이미지 간 변화가 보라색으로 표시되었습니다 (민감도: {self.change_threshold}).\n양쪽 화면에 '변화 후' 이미지를 기반으로 변경사항이 표시됩니다.",
            )

        except Exception as e:
            print(f"detect_pixel_changes 오류: {e}")
            QMessageBox.warning(self, "오류", f"픽셀 변화 감지 중 오류 발생: {e}")
            # 오류 발생 시 변화 감지 모드 해제
            self.change_detection_active = False
            self.change_mask = None


    def reset_images_to_original(self):
        """변화 감지 모드를 해제하고 원본 이미지로 돌아갑니다."""
        try:
            # 변화 감지 모드가 활성화된 경우에만 실행
            if self.change_detection_active:
                # 저장된 원본 QPixmap 이미지가 있으면 복원
                if self.original_before_img and not self.original_before_img.isNull():
                    self.before_box.img = self.original_before_img.copy()
                elif self.selected_a_image_path: # 원본 QPixmap이 없으면 파일에서 다시 로드
                    self.before_box.path = self.selected_a_image_path
                    self.before_box.set_image() # set_image는 내부적으로 repaint 호출
                else: # 경로도 없으면 빈 이미지
                    self.before_box.img = QPixmap()
                    self.before_box.setFixedSize(400,400)


                if self.original_after_img and not self.original_after_img.isNull():
                    self.after_box.img = self.original_after_img.copy()
                elif self.selected_b_image_path:
                    self.after_box.path = self.selected_b_image_path
                    self.after_box.set_image()
                else:
                    self.after_box.img = QPixmap()
                    self.after_box.setFixedSize(400,400)


                # ImageBox 크기 복원 (set_image에서 처리되지만, 명시적으로 호출)
                if self.before_box.img and not self.before_box.img.isNull():
                    self.before_box.w = int(self.before_box.img.width() * self.before_box.scale)
                    self.before_box.h = int(self.before_box.img.height() * self.before_box.scale)
                    self.before_box.setFixedSize(self.before_box.w, self.before_box.h)
                if self.after_box.img and not self.after_box.img.isNull():
                    self.after_box.w = int(self.after_box.img.width() * self.after_box.scale)
                    self.after_box.h = int(self.after_box.img.height() * self.after_box.scale)
                    self.after_box.setFixedSize(self.after_box.w, self.after_box.h)


                # 변수 정리
                self.original_before_img = None
                self.original_after_img = None
                self.change_detection_active = False
                self.change_mask = None

                # 자동 폴리곤 모드도 해제
                if self.auto_polygon_mode:
                    self.auto_polygon_btn.setChecked(False)
                    self.toggle_auto_polygon_mode()


                self.before_box.repaint()
                self.after_box.repaint()

                QMessageBox.information(self, "원본 복원", "원본 이미지로 복원되었습니다.")
            else:
                # 이미 원본 상태이거나, 변화 감지가 실행되지 않은 경우
                # 현재 선택된 이미지로 다시 로드 (사용자가 다른 이미지를 선택했을 수 있으므로)
                if self.selected_a_image_path:
                    self.before_box.path = self.selected_a_image_path
                    self.before_box.set_image()
                if self.selected_b_image_path:
                    self.after_box.path = self.selected_b_image_path
                    self.after_box.set_image()
                QMessageBox.information(self, "알림", "이미 원본 이미지 상태입니다 또는 변화 감지가 실행되지 않았습니다.")


        except Exception as e:
            print(f"reset_images_to_original 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 초기화 실패: {e}")


    def save_no_change(self):
        """
        '변화 없음'으로 라벨링하고, 해당 이미지에 대해 검은색 마스크 이미지를 저장합니다.
        'label_stats/변화없음.txt' 파일에도 파일명을 기록합니다.
        """
        try:
            if not self.selected_a_image_path or not self.selected_b_image_path:
                QMessageBox.information(self, "알림", "저장할 이미지가 선택되지 않았습니다.")
                return

            # 'binary_cd'와 'semantic_cd' 폴더 생성
            current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
            binary_dir = os.path.join(current_dir, 'binary_cd')
            semantic_dir = os.path.join(current_dir, 'semantic_cd')
            label_stat_dir = os.path.join(current_dir, "label_stats")


            os.makedirs(binary_dir, exist_ok=True)
            os.makedirs(semantic_dir, exist_ok=True)
            os.makedirs(label_stat_dir, exist_ok=True)


            # '변화 없음'으로 저장할 때는 보통 'After' 이미지 기준으로 마스크를 생성
            target_image_path = self.selected_b_image_path
            if not target_image_path: # 만약 after 이미지가 없다면 before 이미지 사용
                target_image_path = self.selected_a_image_path

            if not target_image_path: return # 둘 다 없으면 중단

            img_array = np.fromfile(target_image_path, np.uint8)
            img_cv = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if img_cv is None:
                QMessageBox.warning(self, "오류", f"이미지 로드 실패: {target_image_path}")
                return

            height, width = img_cv.shape[:2]
            # 검은색 이미지 생성 (3 채널, BGR 순서)
            black_image_cv = np.zeros((height, width, 3), dtype=np.uint8)

            # 파일명 준비 (보통 After 이미지 파일명을 기준으로 저장)
            base_filename = os.path.splitext(os.path.basename(target_image_path))[0]

            binary_filename = os.path.join(binary_dir, f"{base_filename}.jpg") # 또는 .png
            semantic_filename = os.path.join(semantic_dir, f"{base_filename}.jpg")

            cv2.imwrite(binary_filename, black_image_cv) # OpenCV는 UTF-8 경로 직접 지원
            cv2.imwrite(semantic_filename, black_image_cv)


            # '변화없음.txt' 파일에 파일명 추가 (append_no_change_file 함수 사용)
            append_no_change_file(label_stat_dir, self.selected_a_image_path, self.selected_b_image_path)

            QMessageBox.information(self, "저장 완료", f"'변화 없음' 라벨이 성공적으로 저장되었습니다:\n{binary_filename}\n{semantic_filename}")

        except Exception as e:
            print(f"save_no_change 오류: {e}")
            QMessageBox.warning(self, "오류", f"변화 없음 라벨 저장 실패: {e}")


    def keyPressEvent(self, event):
        """메인 윈도우의 키보드 이벤트 처리"""
        focused_widget = QApplication.focusWidget()

        if event.key() == Qt.Key_W:
            # 자동 포인트 지정 기능 토글
            self.auto_adding_points = not self.auto_adding_points
            if self.auto_adding_points:
                if isinstance(focused_widget, ImageBox) and focused_widget.is_drawing:
                    self.auto_add_timer.start(self.auto_add_interval)
                    print("자동 포인트 추가 시작 (활성 이미지 박스에 그리기 모드일 때)")
                else:
                    self.auto_adding_points = False # 조건 미충족 시 다시 비활성화
                    print("자동 포인트 추가 시작 실패: 활성 이미지 박스에서 그리기 모드가 아닙니다.")
            else:
                self.auto_add_timer.stop()
                print("자동 포인트 추가 중지")

        elif event.key() == Qt.Key_A:
            # 현재 포커스된 ImageBox에 'A' 키 이벤트 전달 (점 추가용)
            if isinstance(focused_widget, ImageBox):
                focused_widget.keyPressEvent(event) # ImageBox의 keyPressEvent 호출
            else: # ImageBox가 포커스되지 않았을 때, 기본 동작 또는 무시
                pass


        # 숫자 1~6번 키에 따른 클래스 변경
        elif Qt.Key_1 <= event.key() <= Qt.Key_6:
            class_num_to_set = event.key() - Qt.Key_0 # 키코드에서 숫자 추출
            self.update_class_from_button(class_num_to_set)

        elif event.key() == Qt.Key_Z and event.modifiers() & Qt.ControlModifier:
            if isinstance(focused_widget, ImageBox) and focused_widget.is_drawing:
                focused_widget.keyPressEvent(event) # ImageBox의 점 단위 undo
            else:
                self.undo() # 메인 윈도우의 폴리곤 단위 undo
        elif event.key() == Qt.Key_Y and event.modifiers() & Qt.ControlModifier:
            if isinstance(focused_widget, ImageBox) and focused_widget.is_drawing:
                focused_widget.keyPressEvent(event) # ImageBox의 점 단위 redo
            else:
                self.redo() # 메인 윈도우의 폴리곤 단위 redo

        # 다음/이전 이미지로 이동 (단축키 예: PageDown, PageUp)
        elif event.key() == Qt.Key_PageDown:
            self.select_next_image()
        elif event.key() == Qt.Key_PageUp:
            self.select_previous_image()

        else:
            # 그 외의 키 이벤트는 포커스된 위젯으로 전달하거나 부모 클래스에서 처리
            if isinstance(focused_widget, (ImageBox, QLineEdit, QListView)): # 특정 위젯들
                # ImageBox의 keyPressEvent는 이미 위에서 Ctrl+Z/Y, A키에 대해 처리했으므로,
                # 여기서는 그 외의 키에 대해서만 전달하거나, ImageBox의 keyPressEvent를 더 정교하게 만들어야 함.
                # 현재는 ImageBox가 포커스되어도 이쪽으로 넘어오면 ImageBox의 다른 키 이벤트는 처리 안 될 수 있음.
                # 해결: ImageBox의 keyPressEvent에서 처리 안 한 이벤트는 super().keyPressEvent(event) 호출하도록 수정함.
                focused_widget.keyPressEvent(event)
            else:
                super().keyPressEvent(event)

    def select_next_image(self):
        """다음 이미지 쌍으로 이동"""
        if not self.temp_listA or not self.LV_A.model(): return
        current_row = self.LV_A.currentIndex().row()
        next_row = current_row + 1
        if 0 <= next_row < self.LV_A.model().rowCount():
            self.LV_A.setCurrentIndex(self.LV_A.model().index(next_row, 0))
            self.on_file_list_selected(self.LV_A.model().index(next_row, 0))

    def select_previous_image(self):
        """이전 이미지 쌍으로 이동"""
        if not self.temp_listA or not self.LV_A.model(): return
        current_row = self.LV_A.currentIndex().row()
        prev_row = current_row - 1
        if 0 <= prev_row < self.LV_A.model().rowCount():
            self.LV_A.setCurrentIndex(self.LV_A.model().index(prev_row, 0))
            self.on_file_list_selected(self.LV_A.model().index(prev_row, 0))


    def add_point_at_cursor_auto(self):
        """자동으로 현재 커서 위치에 포인트를 추가하는 함수 (W 키 토글 타이머에 의해 호출)"""
        if not self.auto_adding_points: # 안전장치
            self.auto_add_timer.stop()
            return

        focused_widget = QApplication.focusWidget()
        if isinstance(focused_widget, ImageBox) and focused_widget.is_drawing:
            # ImageBox의 'A' 키 누름 로직과 유사하게 처리
            pos = focused_widget.mapFromGlobal(QCursor.pos())
            focused_widget.is_moving = False # 이동 중이 아님을 명시
            synthetic_event = QMouseEvent(QEvent.MouseButtonRelease, pos, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
            focused_widget.mouseReleaseEvent(synthetic_event)
        else: # 조건 미충족 시 타이머 중지
            self.auto_adding_points = False
            self.auto_add_timer.stop()
            print("자동 포인트 추가 중지: 조건 미충족")


    def update_class_from_button(self, class_number):
        """클래스 버튼 클릭 시 해당 클래스 번호로 현재 작업 클래스 설정"""
        if 1 <= class_number <= 6:
            self.before_box.current_class = class_number
            self.after_box.current_class = class_number # 두 ImageBox의 현재 클래스 동기화
            self.class_input.setText(str(class_number)) # 입력창에도 반영
            self.highlight_class_button(str(class_number)) # 버튼 하이라이트 업데이트


    def update_class_from_input(self):
        """클래스 입력창에서 엔터 입력 시 현재 작업 클래스 업데이트"""
        try:
            class_number_str = self.class_input.text()
            if not class_number_str.isdigit():
                QMessageBox.warning(self, "입력 오류", "클래스 번호는 숫자여야 합니다.")
                self.class_input.setText(str(self.before_box.current_class)) # 이전 값으로 복원
                return

            class_number = int(class_number_str)
            if 1 <= class_number <= 6:
                self.before_box.current_class = class_number
                self.after_box.current_class = class_number
                self.highlight_class_button(class_number_str)
            else:
                QMessageBox.warning(self, "입력 오류", "클래스 번호는 1에서 6 사이여야 합니다.")
                self.class_input.setText(str(self.before_box.current_class)) # 이전 값으로 복원
        except ValueError:
            QMessageBox.warning(self, "입력 오류", "유효한 숫자를 입력해주세요.")
            self.class_input.setText(str(self.before_box.current_class))
        except Exception as e:
            print(f"update_class_from_input 오류: {e}")
            QMessageBox.warning(self, "오류", f"클래스 업데이트 중 오류 발생: {e}")

    def highlight_class_button(self, class_text):
        """입력된 클래스 번호에 해당하는 버튼을 하이라이트"""
        try:
            current_class_num = int(class_text)
            for num, btn in self.class_buttons.items():
                if num == current_class_num:
                    btn.setStyleSheet("background-color: lightblue; font-weight: bold;")
                else:
                    btn.setStyleSheet("") # 기본 스타일로 복원
        except ValueError: # 숫자가 아닌 경우 모든 버튼 기본 스타일
             for btn in self.class_buttons.values():
                btn.setStyleSheet("")


    def start_single_click_timer_label_list(self, index):
        """라벨 리스트뷰에서 단일 클릭 시 타이머 시작 (더블클릭과 구분 위함)"""
        self.click_index_label_list = index # 클릭된 QModelIndex 저장
        self.click_timer.start(QApplication.doubleClickInterval()) # 시스템 더블클릭 간격 사용

    def handle_single_click_label_list(self):
        """라벨 리스트뷰에서 단일 클릭 처리 (폴리곤 강조)"""
        if self.click_index_label_list and self.click_index_label_list.isValid():
            self.select_polygon_from_list(self.click_index_label_list, double_click=False)

    def handle_double_click_label_list(self, index):
        """라벨 리스트뷰에서 더블 클릭 처리 (클래스 수정)"""
        self.click_timer.stop()  # 단일 클릭 타이머 중지
        if index.isValid():
            self.select_polygon_from_list(index, double_click=True)


    def select_polygon_from_list(self, index, double_click=False):
        """
        라벨 리스트에서 폴리곤 항목을 선택했을 때의 동작.
        - 단일 클릭: 해당 폴리곤을 이미지 위에서 강조.
        - 더블 클릭: 해당 폴리곤의 클래스 수정 다이얼로그 표시.
        """
        try:
            poly_idx_in_list = index.row() # 리스트에서의 인덱스

            if not (0 <= poly_idx_in_list < len(self.before_box.poly_list)):
                return # 유효하지 않은 인덱스

            # 선택된 폴리곤 인덱스를 ImageBox에 전달하여 강조 표시
            self.before_box.selected_poly_index = poly_idx_in_list
            self.after_box.selected_poly_index = poly_idx_in_list # 동기화
            self.before_box.repaint()
            self.after_box.repaint()

            if double_click:
                poly_dict_to_edit = self.before_box.poly_list[poly_idx_in_list]
                current_class_val = poly_dict_to_edit.get('class', self.before_box.current_class)

                new_class_num, ok = QInputDialog.getInt(
                    self, "클래스 수정", "새 클래스 번호 입력 (1~6):",
                    value=current_class_val, min=1, max=6
                )

                if ok:
                    self.push_undo() # 변경 전 상태 저장
                    self.before_box.poly_list[poly_idx_in_list]['class'] = new_class_num
                    # self.after_box.poly_list도 같이 변경됨 (공유하므로)

                    # image_labels 딕셔너리 업데이트
                    if self.before_box.path:
                        self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()

                    self.before_box.repaint()
                    self.after_box.repaint()
                    self.set_list_models() # 라벨 리스트 뷰 내용 업데이트
        except Exception as e:
            print(f"select_polygon_from_list 오류: {e}")
            QMessageBox.warning(self, "오류", f"리스트에서 폴리곤 선택/수정 중 오류 발생: {e}")


    def show_label_list_context_menu(self, point):
        """라벨 리스트뷰 항목 우클릭 시 컨텍스트 메뉴 표시 (삭제 기능)"""
        selected_indexes = self.LV_label.selectedIndexes()
        if not selected_indexes: # 선택된 항목이 없으면 메뉴 표시 안 함
            return

        rightMenu = QMenu(self.LV_label)
        removeAction = QAction("선택한 라벨 삭제", self, triggered=self.remove_selected_polygon_from_list)
        rightMenu.addAction(removeAction)
        rightMenu.exec_(self.LV_label.mapToGlobal(point)) # 클릭 위치에 메뉴 표시

    def remove_selected_polygon_from_list(self):
        """라벨 리스트뷰에서 선택된 폴리곤(들)을 삭제"""
        try:
            selected_indexes = self.LV_label.selectedIndexes()
            if not selected_indexes:
                return

            # 삭제는 역순으로 진행하여 인덱스 꼬임 방지
            rows_to_remove = sorted(list(set(index.row() for index in selected_indexes)), reverse=True)

            if not rows_to_remove: return

            self.push_undo() # 삭제 전 상태 저장

            for row_idx in rows_to_remove:
                if 0 <= row_idx < len(self.before_box.poly_list):
                    self.before_box.poly_list.pop(row_idx)
            # self.after_box.poly_list도 같이 변경됨 (공유)

            # 선택 인덱스 리셋
            self.before_box.selected_poly_index = -1
            self.after_box.selected_poly_index = -1

            # image_labels 딕셔너리 업데이트
            if self.before_box.path:
                self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()

            self.before_box.repaint()
            self.after_box.repaint()
            self.set_list_models() # 라벨 리스트 뷰 업데이트

        except Exception as e:
            print(f"remove_selected_polygon_from_list 오류: {e}")
            QMessageBox.warning(self, "오류", f"폴리곤 삭제 중 오류 발생: {e}")


    def push_undo(self, clear_redo=True):
        """현재 폴리곤 상태를 undo 스택에 저장"""
        # before_box의 poly_list를 기준으로 저장 (어차피 after_box와 공유됨)
        if self.before_box.path: # 현재 작업 중인 이미지가 있을 때만
            # 깊은 복사를 통해 현재 poly_list 상태 저장
            current_polygons_copy = [p.copy() for p in self.before_box.poly_list]
            self.undo_stack.append({
                'path': self.before_box.path, # 어떤 이미지에 대한 상태인지도 저장
                'polygons': current_polygons_copy
            })
            if clear_redo:
                self.redo_stack.clear()


    def undo(self):
        """이전 폴리곤 상태로 되돌리기 (실행 취소)"""
        try:
            if not self.undo_stack:
                QMessageBox.information(self, "실행 취소", "되돌릴 작업이 없습니다.")
                return

            # 현재 상태를 redo 스택에 저장
            if self.before_box.path:
                current_polygons_copy = [p.copy() for p in self.before_box.poly_list]
                self.redo_stack.append({
                    'path': self.before_box.path,
                    'polygons': current_polygons_copy
                })

            # undo 스택에서 이전 상태 복원
            last_state = self.undo_stack.pop()
            restored_path = last_state['path']
            restored_polygons = last_state['polygons']

            # 만약 현재 이미지와 복원할 상태의 이미지가 다르면, 해당 이미지로 전환해야 함.
            # 현재는 같은 이미지 내에서의 undo/redo만 가정.
            if self.before_box.path == restored_path:
                self.before_box.poly_list = [p.copy() for p in restored_polygons] # 복사해서 할당
                # self.after_box.poly_list도 같이 변경됨 (공유)
                self.image_labels[restored_path] = self.before_box.poly_list.copy()

                self.before_box.selected_poly_index = -1
                self.after_box.selected_poly_index = -1
                self.before_box.repaint()
                self.after_box.repaint()
                self.set_list_models()
            else:
                # 다른 이미지의 상태를 현재 이미지에 복원하는 것은 위험하므로, redo 스택에 현재 상태를 다시 넣고 경고.
                self.redo_stack.pop() # 방금 추가한 redo 항목 제거
                self.undo_stack.append(last_state) # undo 스택 원상복구
                QMessageBox.warning(self, "실행 취소 오류", "현재 작업 이미지와 다른 이미지의 상태로는 되돌릴 수 없습니다.")


        except Exception as e:
            print(f"undo 오류: {e}")
            QMessageBox.warning(self, "오류", f"실행 취소 실패: {e}")

    def redo(self):
        """실행 취소된 작업을 다시 실행"""
        try:
            if not self.redo_stack:
                QMessageBox.information(self, "다시 실행", "다시 실행할 작업이 없습니다.")
                return

            # 현재 상태를 undo 스택에 저장 (redo 실행 시에는 clear_redo=False 옵션 필요 없음)
            if self.before_box.path:
                current_polygons_copy = [p.copy() for p in self.before_box.poly_list]
                # redo 실행 시에는 push_undo를 직접 호출하지 않고, redo_stack에서 가져온 것을 적용 후
                # 그 이전 상태(현재 상태)를 undo_stack에 넣는 것이 일반적.
                # 여기서는 push_undo의 clear_redo=False를 활용하는 대신 직접 undo_stack에 추가.
                self.undo_stack.append({
                    'path': self.before_box.path,
                    'polygons': current_polygons_copy
                })


            # redo 스택에서 상태 복원
            next_state = self.redo_stack.pop()
            restored_path = next_state['path']
            restored_polygons = next_state['polygons']

            if self.before_box.path == restored_path:
                self.before_box.poly_list = [p.copy() for p in restored_polygons]
                self.image_labels[restored_path] = self.before_box.poly_list.copy()

                self.before_box.selected_poly_index = -1
                self.after_box.selected_poly_index = -1
                self.before_box.repaint()
                self.after_box.repaint()
                self.set_list_models()
            else:
                self.undo_stack.pop() # 방금 추가한 undo 항목 제거
                self.redo_stack.append(next_state) # redo 스택 원상복구
                QMessageBox.warning(self, "다시 실행 오류", "현재 작업 이미지와 다른 이미지의 상태로는 다시 실행할 수 없습니다.")

        except Exception as e:
            print(f"redo 오류: {e}")
            QMessageBox.warning(self, "오류", f"다시 실행 실패: {e}")


    def on_file_list_selected(self, qModelIndex):
        """파일 리스트(LV_A 또는 LV_B)에서 항목 선택 시 호출"""
        try:
            # 어떤 리스트뷰에서 이벤트가 발생했는지 확인
            sender_list_view = self.sender()
            if not isinstance(sender_list_view, QListView):
                return

            selected_row = qModelIndex.row()

            # 현재 작업 중인 라벨 저장
            if self.before_box.path and self.before_box.poly_list:
                self.image_labels[self.before_box.path] = [p.copy() for p in self.before_box.poly_list]

            # undo 및 redo 스택 초기화 (새 이미지 쌍 로드 시)
            self.undo_stack.clear()
            self.redo_stack.clear()

            # 선택된 행에 해당하는 이미지 경로 가져오기
            if 0 <= selected_row < len(self.temp_listA) and 0 <= selected_row < len(self.temp_listB):
                new_a_path = self.temp_listA[selected_row]
                new_b_path = self.temp_listB[selected_row]

                # 이전에 선택된 이미지와 다를 경우에만 이미지 로드 및 폴리곤 초기화
                if self.selected_a_image_path != new_a_path or self.selected_b_image_path != new_b_path:
                    self.selected_a_image_path = new_a_path
                    self.selected_b_image_path = new_b_path

                    self.before_box.path = self.selected_a_image_path
                    self.before_box.set_image() # 내부에서 repaint 호출

                    self.after_box.path = self.selected_b_image_path
                    self.after_box.set_image() # 내부에서 repaint 호출

                    # 새 이미지에 대한 라벨 로드 또는 초기화
                    if self.before_box.path in self.image_labels:
                        self.before_box.poly_list = [p.copy() for p in self.image_labels[self.before_box.path]]
                    else:
                        self.before_box.poly_list = []

                    # after_box는 before_box의 poly_list를 공유
                    self.after_box.poly_list = self.before_box.poly_list

                    # 변화 감지 모드 관련 변수 초기화
                    self.change_detection_active = False
                    self.change_mask = None
                    self.original_before_img = None
                    self.original_after_img = None
                    if self.auto_polygon_mode: # 이미지 변경 시 자동 폴리곤 모드 해제
                        self.auto_polygon_btn.setChecked(False)
                        self.toggle_auto_polygon_mode()


                    self.before_box.selected_poly_index = -1 # 선택 폴리곤 초기화
                    self.after_box.selected_poly_index = -1

                    self.before_box.repaint() # 명시적 repaint
                    self.after_box.repaint()
                    self.set_list_models() # 라벨 리스트 및 파일 리스트 모델 업데이트

                    # 이미지 선택 후 바로 픽셀 변화 감지 실행
                    if self.selected_a_image_path and self.selected_b_image_path:
                        # 사용자가 원하지 않을 경우를 대비해, 옵션으로 제공하는 것이 더 좋을 수 있음
                        # 여기서는 요청대로 바로 실행
                        print("이미지 선택 완료, 자동 픽셀 변화 감지 실행...")
                        self.detect_pixel_changes()


                # 양쪽 리스트뷰의 선택 동기화
                if sender_list_view == self.LV_A:
                    self.LV_B.setCurrentIndex(self.LV_B.model().index(selected_row, 0))
                elif sender_list_view == self.LV_B:
                    self.LV_A.setCurrentIndex(self.LV_A.model().index(selected_row, 0))

            else:
                # QMessageBox.warning(self, "오류", "선택한 파일 인덱스가 유효하지 않습니다.")
                print(f"선택한 파일 인덱스 {selected_row}가 유효하지 않습니다.")


        except Exception as e:
            print(f"on_file_list_selected 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 쌍 로드 실패: {e}")

    def open_folder_dialog(self, list_type):
        """
        특정 이미지를 선택하면 해당 이미지부터 아래 100개 파일만 temp_listA / temp_listB에 절대경로로 할당합니다.
        """
        try:
            # 기존 작업 중이던 라벨 저장
            if self.before_box.path and self.before_box.poly_list:
                self.image_labels[self.before_box.path] = [p.copy() for p in self.before_box.poly_list]

            # 1) 파일 선택 대화창(단일 파일) 열기
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                f"{list_type} 이미지 파일 선택",
                "",
                "Images (*.png *.jpg *.jpeg *.bmp)"
            )
            if not file_path:
                return

            # 절대경로로 변환
            file_path = os.path.abspath(file_path)

            # 2) 선택한 파일이 있는 폴더에서 모든 이미지 수집 및 정렬
            folder_path = os.path.dirname(file_path)
            folder_path = os.path.abspath(folder_path)

            all_images = [
                os.path.abspath(os.path.join(folder_path, f))
                for f in os.listdir(folder_path)
                if f.lower().endswith((".png", ".jpg", ".jpeg", ".bmp"))
            ]
            all_images.sort(key=natural_key)

            # 3) 선택한 파일의 인덱스 찾고, 그 이후 100개만 슬라이싱
            try:
                start_idx = all_images.index(file_path)
            except ValueError:
                QMessageBox.warning(self, "오류", "선택한 파일을 폴더에서 찾을 수 없습니다.")
                return

            subset_images = all_images[start_idx:start_idx + 100]

            # 4) list_type에 따라 temp_listA 또는 temp_listB 갱신
            if list_type == "A":
                self.temp_listA = subset_images
            elif list_type == "B":
                self.temp_listB = subset_images

            # 5) 리스트뷰 갱신
            self.set_list_models()

            # 6) A, B 리스트가 모두 로드된 상태라면 첫 쌍을 자동 선택
            if self.temp_listA and self.temp_listB and len(self.temp_listA) == len(self.temp_listB):
                first_index = self.LV_A.model().index(0, 0)
                if first_index.isValid():
                    self.LV_A.setCurrentIndex(first_index)
                    self.on_file_list_selected(first_index)
            else:
                # 개수가 다르면 경고
                countA = len(self.temp_listA) if hasattr(self, 'temp_listA') else 0
                countB = len(self.temp_listB) if hasattr(self, 'temp_listB') else 0
                QMessageBox.warning(
                    self, "경고",
                    f"Before/After 이미지 수가 다릅니다.\nBefore: {countA}개, After: {countB}개"
                )

        except Exception as e:
            print(f"open_folder_dialog 오류 ({list_type}): {e}")
            QMessageBox.warning(self, "오류", f"이미지 파일을 여는 데 실패했습니다: {e}")

    def savepoint(self):
        """
        현재까지 라벨링된 모든 이미지의 폴리곤 정보를 기반으로
        바이너리 마스크와 시맨틱 마스크 이미지를 생성하여 저장합니다.
        'binary_cd' 및 'semantic_cd' 폴더에 저장됩니다.
        또한, 'label_stats' 폴더에 통계 정보를 저장합니다.
        """
        try:
            # 현재 작업 중인 이미지의 라벨을 image_labels에 마지막으로 업데이트
            if self.before_box.path and self.before_box.poly_list:
                self.image_labels[self.before_box.path] = [p.copy() for p in self.before_box.poly_list]

            if not self.image_labels:
                QMessageBox.information(self, "알림", "저장할 라벨 데이터가 없습니다.")
                return

            # 저장 폴더 경로 설정
            current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
            binary_dir = os.path.join(current_dir, 'binary_cd')
            semantic_dir = os.path.join(current_dir, 'semantic_cd')
            label_stat_dir = os.path.join(current_dir, "label_stats")

            os.makedirs(binary_dir, exist_ok=True)
            os.makedirs(semantic_dir, exist_ok=True)
            os.makedirs(label_stat_dir, exist_ok=True)

            saved_count = 0
            for image_path, poly_list_for_image in self.image_labels.items():
                if not poly_list_for_image: # 해당 이미지에 폴리곤이 없으면 건너뛰기
                    continue

                # 원본 이미지 로드 (크기 정보 얻기 위함)
                # image_path는 before 이미지 경로 기준
                img_array = np.fromfile(image_path, np.uint8)
                original_cv_img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if original_cv_img is None:
                    print(f"원본 이미지 로드 실패 (저장 건너뜀): {image_path}")
                    continue

                height, width = original_cv_img.shape[:2]

                # 바이너리 마스크 (변화 영역: 흰색, 배경: 검은색)
                binary_mask_cv = np.zeros((height, width), dtype=np.uint8)
                # 시맨틱 마스크 (클래스별 색상, 배경: 검은색)
                semantic_mask_cv = np.zeros((height, width, 3), dtype=np.uint8)


                for poly_dict in poly_list_for_image:
                    points_relative = poly_dict.get('points', [])
                    class_number = poly_dict.get('class', 0) # 클래스 없으면 0 (검은색)

                    if len(points_relative) < 6: continue # 최소 3개의 점 필요

                    # 상대 좌표를 절대 정수 좌표로 변환 (OpenCV 폴리곤 함수는 정수 좌표 사용)
                    polygon_points_cv = []
                    for i in range(0, len(points_relative), 2):
                        # 이미지 원본 기준 좌표이므로 스케일이나 오프셋 적용 불필요
                        # 단, 소수점일 수 있으므로 int로 변환
                        px, py = int(points_relative[i]), int(points_relative[i+1])
                        polygon_points_cv.append([px, py])

                    if not polygon_points_cv: continue

                    np_poly_pts = np.array([polygon_points_cv], dtype=np.int32)

                    # 바이너리 마스크에 폴리곤 채우기 (흰색: 255)
                    cv2.fillPoly(binary_mask_cv, np_poly_pts, 255)

                    # 시맨틱 마스크에 클래스별 색상으로 폴리곤 채우기 (BGR 순서)
                    # ImageBox의 get_class_color는 QColor 반환, OpenCV용 BGR 값 필요
                    color_bgr = (0,0,0) # 기본 검정
                    if class_number == 1: color_bgr = (0, 255, 255)  # 사람 - 노란색 (BGR)
                    elif class_number == 2: color_bgr = (128, 128, 128) # 돌 - 회색
                    elif class_number == 3: color_bgr = (19, 69, 139)   # 흙 - 갈색
                    elif class_number == 4: color_bgr = (255, 0, 0)     # 물 - 파란색
                    elif class_number == 5: color_bgr = (0, 0, 255)     # 불 - 빨간색
                    elif class_number == 6: color_bgr = (0, 128, 0)     # 나무 - 초록색
                    cv2.fillPoly(semantic_mask_cv, np_poly_pts, color_bgr)

                # 파일명 준비 ( 원본 이미지 파일명 기반)
                base_filename = os.path.splitext(os.path.basename(image_path))[0]
                binary_save_path = os.path.join(binary_dir, f"{base_filename}.png") # PNG로 저장 권장
                semantic_save_path = os.path.join(semantic_dir, f"{base_filename}.png")

                cv2.imwrite(binary_save_path, binary_mask_cv)
                cv2.imwrite(semantic_save_path, semantic_mask_cv)
                saved_count +=1

            if saved_count > 0:
                QMessageBox.information(self, "저장 완료", f"{saved_count}개의 이미지에 대한 라벨 마스크가 성공적으로 저장되었습니다.")
                # 라벨 통계 저장
                save_label_statistics(self.image_labels, label_stat_dir)
            else:
                QMessageBox.information(self, "알림", "저장할 유효한 라벨이 있는 이미지가 없습니다.")

            self.set_list_models() # 저장 후 리스트 뷰 갱신 (필요 시)

        except Exception as e:
            print(f"savepoint 오류: {e}\n{traceback.format_exc()}")
            QMessageBox.critical(self, "저장 오류", f"라벨 저장 중 심각한 오류가 발생했습니다: {e}")


    def load_labels_from_external_file_if_exists(self, image_path_key):
        """
        (선택적 기능) 이미지에 해당하는 외부 라벨 파일(예: CSV, JSON)이 있다면 로드.
        현재 코드에서는 사용되지 않음. 필요 시 구현.
        """
        pass


    def set_list_models(self):
        """
        파일 리스트(LV_A, LV_B)와 라벨 리스트(LV_label)의 모델을 설정/업데이트합니다.
        선택된 파일은 굵게 표시됩니다.
        """
        try:
            # 파일 리스트 모델 업데이트
            model_a = QStandardItemModel()
            for file_path in self.temp_listA:
                item = QStandardItem(os.path.basename(file_path))
                if file_path == self.selected_a_image_path:
                    font = item.font(); font.setBold(True); item.setFont(font)
                model_a.appendRow(item)
            self.LV_A.setModel(model_a)

            model_b = QStandardItemModel()
            for file_path in self.temp_listB:
                item = QStandardItem(os.path.basename(file_path))
                if file_path == self.selected_b_image_path:
                    font = item.font(); font.setBold(True); item.setFont(font)
                model_b.appendRow(item)
            self.LV_B.setModel(model_b)

            # 라벨 리스트 모델 업데이트 (현재 before_box의 poly_list 기준)
            poly_info_for_list = []
            class_names_map = {1: "사람", 2: "돌", 3: "흙", 4: "물", 5: "불", 6: "나무"}
            for idx, poly_dict in enumerate(self.before_box.poly_list):
                class_num = poly_dict.get('class', 0)
                class_name_str = class_names_map.get(class_num, f"미지정({class_num})")
                # points_str = str(poly_dict.get('points', [])) # 너무 길어질 수 있음
                # 간단히 "폴리곤 [인덱스]: 클래스명" 형태로 표시
                poly_info_for_list.append(f"폴리곤 {idx + 1}: {class_name_str}")

            label_model = QStringListModel(poly_info_for_list)
            self.LV_label.setModel(label_model)

        except Exception as e:
            print(f"set_list_models 오류: {e}")


    def closeEvent(self, event):
        """윈도우 종료 시 확인 질문 및 자동 저장 기능 (선택 사항)"""
        reply = QMessageBox.question(
            self,
            "프로그램 종료",
            "변경사항을 저장하지 않고 종료하시겠습니까?\n('라벨 저장' 메뉴를 통해 수동 저장을 권장합니다)",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No, # 기본 선택 No
        )
        if reply == QMessageBox.Yes:
            if self.auto_adding_points: # 자동 포인트 추가 타이머 중지
                self.auto_add_timer.stop()
            event.accept() # 종료 수락
        else:
            event.ignore() # 종료 무시


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = change_detection()
    win.show()
    win.setFocus() # 메인 윈도우에 초기 포커스 설정
    sys.exit(app.exec_())