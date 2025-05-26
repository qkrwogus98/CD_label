import pandas as pd
import cv2
import numpy as np
import os
import sys
import traceback
from PyQt5.QtCore import (
    Qt, QPoint, QEvent, QTimer, QStringListModel, QSize, QRect
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
    QGroupBox,
    QRadioButton
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

        if os.path.exists(txt_path):
            with open(txt_path, "r", encoding="utf-8") as f:
                existing_files = set(line.strip() for line in f.readlines())

        all_files = existing_files.union(files)

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

    if filename != os.path.basename(selected_b_image_path):
        print("[경고] before와 after 파일명이 다릅니다. 저장 안 함.")
        return

    if os.path.exists(no_change_file):
        with open(no_change_file, "r", encoding="utf-8") as f:
            existing = set(line.strip() for line in f.readlines())
            if filename in existing:
                return
    with open(no_change_file, "a", encoding="utf-8") as f:
        f.write(filename + "\n")

def exception_hook(exctype, value, tb):
    tb_msg = ''.join(traceback.format_exception(exctype, value, tb))
    print(tb_msg) 
    QMessageBox.critical(None, "오류 발생", f"프로그램 실행 중 오류가 발생했습니다:\n\n{tb_msg}\n\n자세한 내용은 콘솔 출력을 확인해주세요.")

sys.excepthook = exception_hook

class ImageBox(QWidget):
    def __init__(self, is_after=False):
        super(ImageBox, self).__init__()
        self.is_after = is_after
        self.poly_list = []
        self.path = None
        self.scale = 1.0
        self.w = None
        self.h = None
        self.point = QPoint(0, 0)
        self.img = None
        self.start_pos = None
        self.end_pos = None
        self.is_left_clicked = False
        self.is_right_clicked = False # 우클릭 상태 플래그 추가
        self.is_moving = False
        self.setCursor(Qt.PointingHandCursor)
        self.is_drawing = False
        self.line = []
        self.pos = None
        self.is_closed = False
        self.current_class = 1
        self.selected_poly_index = -1
        self.setFocusPolicy(Qt.ClickFocus)
        self.line_redo_stack = []
        self.pair_box = None
        self.bigbox = None
        self.shape_cursor_type = None
        self.shape_cursor_size = 50
        self.show_shape_preview = False

    def set_pair_box(self, box):
        self.pair_box = box

    def set_region_tool_params(self, tool_type, tool_size, region_mode_active):
        self.shape_cursor_type = tool_type if region_mode_active else None
        self.shape_cursor_size = tool_size
        self.show_shape_preview = region_mode_active and (tool_type in ["square", "circle"])
        if not self.show_shape_preview:
            pass
        else:
            self.is_drawing = False 
            self.line = []
        self.update()

    def handle_polygon_class_input(self, pos):
        try:
            for index, poly_dict in enumerate(self.poly_list):
                points = poly_dict['points']
                if len(points) < 4 or len(points) % 2 != 0: continue
                scaled_points = [QPoint(int(x), int(y)) for i in range(0, len(points), 2) for x, y in [self.get_absolute_coor([[points[i], points[i+1]]])]]
                poly = QPolygon(scaled_points)
                if poly.containsPoint(pos, Qt.OddEvenFill):
                    class_number, ok = QInputDialog.getInt(self, "클래스 수정", "클래스 번호 입력 (1~6):", value=poly_dict.get('class', self.current_class), min=1, max=6)
                    if ok:
                        self.poly_list[index]['class'] = class_number
                        if self.bigbox and self.path: self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        self.update()
                        if self.pair_box: self.pair_box.update()
                    break
        except Exception as e:
            print(f"handle_polygon_class_input 오류: {e}")
            QMessageBox.warning(self, "오류", f"폴리곤 클래스 입력 중 오류 발생: {e}")

    def mouseDoubleClickEvent(self, e):
        try:
            if self.bigbox and self.bigbox.auto_polygon_mode: return
            click_pos = e.pos()
            for index, poly_dict in enumerate(self.poly_list):
                points = poly_dict['points']
                if len(points) < 4 or len(points) % 2 != 0: continue
                scaled_points = [QPoint(int(x), int(y)) for i in range(0, len(points), 2) for x,y in [self.get_absolute_coor([[points[i], points[i+1]]])]]
                poly = QPolygon(scaled_points)
                if poly.containsPoint(click_pos, Qt.OddEvenFill):
                    class_number, ok = QInputDialog.getInt(self, "클래스 수정", "새 클래스 번호 입력 (1~6):", value=poly_dict.get('class', self.current_class), min=1, max=6)
                    if ok:
                        if self.bigbox: self.bigbox.push_undo()
                        self.poly_list[index]['class'] = class_number
                        if self.bigbox and self.path: self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        self.update()
                        if self.pair_box: self.pair_box.update()
                        if self.bigbox: self.bigbox.set_list_models()
                    break
        except Exception as e:
            print(f"mouseDoubleClickEvent 오류: {e}")
            QMessageBox.warning(self, "오류", f"더블 클릭 이벤트 처리 중 오류 발생: {e}")

    def set_image(self):
        try:
            if not self.path or not os.path.exists(self.path):
                self.img = None; self.setFixedSize(400,400); self.update(); return
            img_array = np.fromfile(self.path, np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if img is None: raise ValueError(f"cv2.imdecode가 이미지를 로드하지 못했습니다: {self.path}")
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            height, width, channel = img.shape
            qimg = QImage(img.data, width, height, 3 * width, QImage.Format_RGB888)
            self.img = QPixmap.fromImage(qimg)
            if hasattr(self.bigbox, "change_detection_active") and self.bigbox.change_detection_active: pass
        except Exception as e:
            print(f"이미지 로드 오류 ({self.path}): {e}")
            self.img = QPixmap(); QMessageBox.warning(self, "이미지 로드 오류", f"이미지를 불러오는 데 실패했습니다: {self.path}\n{e}")
        try:
            self.scale = 1.0
            if self.img and not self.img.isNull():
                self.w = self.img.width(); self.h = self.img.height()
                self.setMinimumSize(100, 100); self.setFixedSize(self.w, self.h)
            else:
                self.w = 400; self.h = 400; self.setFixedSize(self.w, self.h)
            self.point = QPoint(0, 0); self.update()
        except Exception as e:
            print(f"set_image 크기 설정 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 설정 실패: {e}")

    def paintEvent(self, e):
        try:
            painter = QPainter(); painter.begin(self)
            if self.img and not self.img.isNull():
                painter.drawPixmap(int(self.point.x()), int(self.point.y()), int(self.w), int(self.h), self.img)
                for index, poly_dict in enumerate(self.poly_list):
                    points = poly_dict.get('points', []); class_number = poly_dict.get('class', self.current_class)
                    pen = QPen(self.get_class_color(class_number)); pen.setWidth(3)
                    if index == self.selected_poly_index: pen.setWidth(5)
                    painter.setPen(pen); painter.setBrush(QBrush(self.get_class_color(class_number, alpha=50)))
                    if len(points) < 4 or len(points) % 2 != 0: continue
                    poly_qpolygon = QPolygon()
                    for i in range(0, len(points), 2):
                        abs_coords = self.get_absolute_coor([[points[i], points[i + 1]]])
                        if len(abs_coords) >= 2:
                            x, y = abs_coords[0], abs_coords[1]
                            if not (np.isnan(x) or np.isnan(y)): poly_qpolygon.append(QPoint(int(x), int(y)))
                    if poly_qpolygon.size() > 1: painter.drawPolygon(poly_qpolygon)

                if self.is_drawing and self.pos and self.line:
                    pen = QPen(self.get_class_color(self.current_class)); pen.setWidth(3)
                    painter.setPen(pen); painter.setBrush(Qt.NoBrush)
                    if len(self.line) >=2:
                        start_coords = self.get_absolute_coor([[self.line[0], self.line[1]]])
                        if len(start_coords) >=2: painter.drawEllipse(QPoint(int(start_coords[0]), int(start_coords[1])), 5, 5)
                    if self.is_closed and len(self.line) >=2:
                        end_coords = self.get_absolute_coor([[self.line[0], self.line[1]]])
                        if len(end_coords) >=2: painter.drawEllipse(QPoint(int(end_coords[0]), int(end_coords[1])), 10, 10)
                    for i in range(0, len(self.line) - 2, 2):
                        p1_coords = self.get_absolute_coor([[self.line[i], self.line[i+1]]]); p2_coords = self.get_absolute_coor([[self.line[i+2], self.line[i+3]]])
                        if len(p1_coords) >=2 and len(p2_coords) >=2 and not any(np.isnan(c) for c in p1_coords+p2_coords):
                            painter.drawLine(int(p1_coords[0]), int(p1_coords[1]), int(p2_coords[0]), int(p2_coords[1]))
                    if self.pos and len(self.line) >=2:
                        last_point_coords = self.get_absolute_coor([[self.line[-2], self.line[-1]]])
                        if len(last_point_coords) >= 2 and not any(np.isnan(c) for c in last_point_coords):
                            painter.drawLine(int(last_point_coords[0]), int(last_point_coords[1]), int(self.pos.x()), int(self.pos.y()))
                
                if self.bigbox and self.bigbox.region_select_mode and self.show_shape_preview and self.pos:
                    painter.setPen(QPen(QColor(0, 255, 0, 180), 2, Qt.DashLine)); painter.setBrush(Qt.NoBrush)
                    size = self.shape_cursor_size; half_size = size // 2
                    preview_rect = QRect(self.pos.x() - half_size, self.pos.y() - half_size, size, size)
                    if self.shape_cursor_type == "square": painter.drawRect(preview_rect)
                    elif self.shape_cursor_type == "circle": painter.drawEllipse(preview_rect)
            else:
                painter.fillRect(self.rect(), QColor(200, 200, 200)); painter.setPen(Qt.black)
                painter.drawText(self.rect(), Qt.AlignCenter, "이미지를 불러오세요.")
            font = QFont(); font.setPointSize(14); painter.setFont(font)
            painter.setPen(QPen(Qt.red if self.is_after else Qt.blue))
            painter.drawText(10, 30, "변화 후 (After Change)" if self.is_after else "변화 전 (Before Change)")
            painter.end()
        except Exception as e:
            print(f"paintEvent 오류: {e}")
            if painter.isActive(): painter.end()

    def get_class_color(self, class_number, alpha=255):
        colors = {1: (255,255,0), 2: (128,128,128), 3: (139,69,19), 4: (0,0,255), 5: (255,0,0), 6: (0,128,0)}
        r,g,b = colors.get(class_number, (0,0,0))
        return QColor(r,g,b,alpha)

    def get_absolute_coor(self, coord_list):
        return [self.point.x() + self.scale * c[0] for c in coord_list] + [self.point.y() + self.scale * c[1] for c in coord_list]

    def get_relative_coor(self, abs_pos_widget):
        if self.scale == 0: return QPoint(0,0)
        return QPoint(int((abs_pos_widget.x() - self.point.x()) / self.scale), int((abs_pos_widget.y() - self.point.y()) / self.scale))

    def mouseMoveEvent(self, e: QMouseEvent):
        try:
            self.pos = e.pos() 
            is_auto_poly_mode = self.bigbox and self.bigbox.auto_polygon_mode
            
            # 우클릭 드래그로 이미지 이동 (팬 모드)
            if self.is_right_clicked and self.start_pos: # is_right_clicked는 press에서, start_pos는 press에서 설정
                self.end_pos = e.pos() - self.start_pos
                old_x, old_y = self.point.x(), self.point.y()
                self.point = self.point + self.end_pos
                self.start_pos = e.pos() # 다음 이동을 위해 시작점 갱신
                self.is_moving = True # 이동 중임을 표시
                if self.pair_box and not self.pair_box.is_right_clicked: # 상대방이 우클릭 이동 중이 아닐 때만 동기화
                    dx = self.point.x() - old_x; dy = self.point.y() - old_y
                    self.pair_box.point = QPoint(self.pair_box.point.x() + dx, self.pair_box.point.y() + dy)
                    self.pair_box.update()
            
            elif self.is_drawing: # 일반 폴리곤 그리기 중 (좌클릭 기반)
                if len(self.line) >= 2:
                    coords = self.get_absolute_coor([[self.line[0], self.line[1]]])
                    if len(coords) >= 2:
                        x1_abs, y1_abs = coords[0], coords[1]
                        self.is_closed = (abs(self.pos.x() - x1_abs) < 10 and abs(self.pos.y() - y1_abs) < 10 and len(self.line) > 4)
                    else: self.is_closed = False
                else: self.is_closed = False
            
            self.update()
        except Exception as err:
            print(f"mouseMoveEvent 오류: {err}"); traceback.print_exc()

    def mousePressEvent(self, e):
        self.setFocus()
        if e.button() == Qt.LeftButton:
            self.is_left_clicked = True
            self.start_pos = e.pos() # 좌클릭 시작점 (그리기/도형 선택용)
            self.is_moving = False # 좌클릭 시에는 이동 모드 아님
            if self.bigbox and self.bigbox.auto_polygon_mode and self.bigbox.change_mask is not None:
                self.try_auto_create_polygon(e.pos())
                return 
        elif e.button() == Qt.RightButton:
            self.is_right_clicked = True
            self.start_pos = e.pos() # 우클릭 시작점 (이동용)
            self.is_moving = False

    def mouseReleaseEvent(self, e: QMouseEvent):
        try:
            # 우클릭 해제: 이동 종료 또는 그리기 메뉴
            if e.button() == Qt.RightButton:
                if not self.is_moving: # 이동 중이 아니었다면 (단순 우클릭)
                    if self.is_drawing and (not self.bigbox or not self.bigbox.region_select_mode or self.bigbox.current_region_selection_tool_type == "polygon"):
                        menu = QMenu(self)
                        menu.addAction("완료", lambda: self.update_line(None, "finish"))
                        menu.addAction("취소", lambda: self.update_line(None, "cancel"))
                        menu.exec_(QCursor.pos())
                # is_moving 상태와 관계없이 우클릭 상태는 해제
                self.is_right_clicked = False
                self.is_moving = False # 이동 상태도 여기서 확실히 리셋
                self.start_pos = None # 이동 시작점 리셋
                return

            if e.button() != Qt.LeftButton: return

            # --- 팬 모드 종료 (좌클릭으로는 팬 안함) ---
            # if self.is_moving: # 이 부분은 우클릭으로 옮겨졌으므로 제거 또는 주석처리
            #     self.is_moving = False
            #     self.is_left_clicked = False # 좌클릭 상태도 여기서 리셋
            #     return

            if self.bigbox and self.bigbox.auto_polygon_mode and self.bigbox.change_mask is not None:
                self.is_left_clicked = False; return

            if self.bigbox and self.bigbox.region_select_mode and \
               self.bigbox.current_region_selection_tool_type in ["square", "circle"] and \
               self.bigbox.change_mask is not None:
                click_pos_widget = e.pos()
                shape_type = self.bigbox.current_region_selection_tool_type
                shape_size_on_widget = self.shape_cursor_size
                half_size_widget = shape_size_on_widget // 2
                widget_tl = QPoint(click_pos_widget.x() - half_size_widget, click_pos_widget.y() - half_size_widget)
                widget_br = QPoint(click_pos_widget.x() + half_size_widget, click_pos_widget.y() + half_size_widget)
                rel_tl = self.get_relative_coor(widget_tl); rel_br = self.get_relative_coor(widget_br)
                poly_pts = [float(rel_tl.x()), float(rel_tl.y()), float(rel_br.x()), float(rel_tl.y()),
                            float(rel_br.x()), float(rel_br.y()), float(rel_tl.x()), float(rel_br.y())]
                if len(poly_pts) >= 6: self.handle_region_selection(poly_pts)
                else: print("도형 영역 선택 좌표 계산 오류")
                self.is_left_clicked = False; self.update(); return
            
            can_draw_polygon_manually = False
            if self.bigbox:
                if self.bigbox.region_select_mode and self.bigbox.current_region_selection_tool_type == "polygon": can_draw_polygon_manually = True
                elif not self.bigbox.region_select_mode and not self.bigbox.auto_polygon_mode: can_draw_polygon_manually = True
            
            if can_draw_polygon_manually:
                rel = self.get_relative_coor(e.pos())
                if not self.is_drawing:
                    self.is_drawing = True; self.line = []; self.line_redo_stack.clear()
                    self.update_line(rel, "draw")
                else:
                    if self.line:
                        abs_start_coords = self.get_absolute_coor([[self.line[0], self.line[1]]])
                        if len(abs_start_coords) >= 2:
                            abs_sx, abs_sy = abs_start_coords[0], abs_start_coords[1]
                            if abs(e.pos().x() - abs_sx) < 10 and abs(e.pos().y() - abs_sy) < 10 and len(self.line) > 4:
                                self.update_line(None, "finish")
                            else: self.update_line(rel, "draw")
                        else: self.update_line(rel, "draw")
                    else: self.is_drawing = False
            self.is_left_clicked = False
        except Exception as exc:
            print(f"mouseReleaseEvent 오류: {exc}"); traceback.print_exc()
            QMessageBox.warning(self, "오류", f"마우스 릴리즈 처리 중 오류 발생:\n{exc}")
        finally:
            self.is_left_clicked = False
            self.is_right_clicked = False # 우클릭 상태도 확실히 리셋
            self.is_moving = False # 이동 상태도 확실히 리셋
            # self.start_pos = None # start_pos는 다음 클릭을 위해 유지될 수 있음 (상황에 따라)

    def try_auto_create_polygon(self, click_pos_widget):
        # ... (기존 코드와 동일하게 유지)
        print(f"자동 폴리곤 생성 시도: {click_pos_widget}")
        if not self.bigbox or not self.bigbox.change_mask is not None:
            print("자동 폴리곤 생성 실패: 변화 감지 마스크 없음.")
            return

        click_pt_on_mask_qpoint = self.get_relative_coor(click_pos_widget)
        click_pt_on_mask = (click_pt_on_mask_qpoint.x(), click_pt_on_mask_qpoint.y())

        mask_h, mask_w = self.bigbox.change_mask.shape[:2]
        if not (0 <= click_pt_on_mask[0] < mask_w and 0 <= click_pt_on_mask[1] < mask_h):
            print("클릭 위치가 마스크 범위를 벗어났습니다.")
            return
        if self.bigbox.change_mask[click_pt_on_mask[1], click_pt_on_mask[0]] == 0:
            print("클릭 위치는 변화가 감지된 영역이 아닙니다.")
            return

        contours, _ = cv2.findContours(self.bigbox.change_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("마스크에서 contour를 찾지 못했습니다.")
            return

        selected_contour = None
        for contour in contours:
            if cv2.pointPolygonTest(contour, (click_pt_on_mask[0], click_pt_on_mask[1]), False) >= 0:
                selected_contour = contour
                break
        if selected_contour is None:
            print("클릭 위치를 포함하는 contour를 찾지 못했습니다.")
            return

        polygon_points_cv = selected_contour.reshape(-1, 2)
        new_line = [float(pt[c]) for pt in polygon_points_cv for c in range(2)]

        if len(new_line) < 6:
            print("자동 생성된 폴리곤의 점 개수가 너무 적습니다.")
            return

        if self.bigbox: self.bigbox.push_undo()
        self.poly_list.append({'points': new_line.copy(), 'class': self.current_class})

        if self.bigbox and self.path:
            self.bigbox.image_labels[self.path] = self.poly_list.copy()
            self.bigbox.redo_stack.clear()
            self.bigbox.set_list_models()

        self.is_drawing = False 
        self.is_closed = False
        self.update()
        if self.pair_box: self.pair_box.update()
        print("자동 폴리곤 생성 완료!")


    def update_line(self, rel_pos=None, flag="draw"):
        # ... (기존 코드와 거의 동일, repaint() -> update() 변경 정도)
        try:
            if flag == "cancel":
                self.line = []; self.line_redo_stack.clear(); self.is_drawing = False; self.is_closed = False; self.update(); return

            if flag == "finish" and self.bigbox and self.bigbox.region_select_mode and self.bigbox.current_region_selection_tool_type == "polygon":
                if len(self.line) >= 6: self.handle_region_selection(self.line)
                else: QMessageBox.information(self, "알림", "영역선택을 위해 최소 3개의 점으로 폴리곤을 그려주세요.")
                self.line = []; self.is_drawing = False; self.is_closed = False; self.update(); return

            if flag == "finish":
                if len(self.line) > 4:
                    self.poly_list.append({'points': self.line.copy(), 'class': self.current_class})
                    if self.bigbox and self.path:
                        self.bigbox.push_undo()
                        self.bigbox.image_labels[self.path] = self.poly_list.copy()
                        self.bigbox.redo_stack.clear()
                        self.bigbox.set_list_models()
                else: self.update_line(None, "cancel"); return
                self.line = []; self.is_drawing = False; self.is_closed = False; self.update()
                if hasattr(self, 'pair_box') and self.pair_box: self.pair_box.update()
                return

            if flag == "draw" and rel_pos is not None:
                if self.is_closed: self.update_line(None, "finish")
                else:
                    self.line.extend([rel_pos.x(), rel_pos.y()])
                    self.line_redo_stack.clear(); self.update()
                return
        except Exception as e: print(f"update_line 오류: {e}"); traceback.print_exc()
            
    def handle_region_selection(self, poly_pts_relative):
        # ... (기존 코드와 거의 동일, repaint() -> update() 변경 정도)
        if not poly_pts_relative or len(poly_pts_relative) < 6: print("handle_region_selection: 유효하지 않은 poly_pts_relative"); return
        mask = self.bigbox.change_mask
        if mask is None: QMessageBox.warning(self, "오류", "먼저 픽셀 변화 감지를 실행하세요."); return
        roi = np.zeros_like(mask)
        pts_for_cv = np.array(poly_pts_relative, dtype=np.float32).reshape(-1, 2).astype(np.int32)
        cv2.fillPoly(roi, [pts_for_cv], 255)
        sel = cv2.bitwise_and(mask, mask, mask=roi)
        contours, _ = cv2.findContours(sel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            QMessageBox.information(self, "알림", "선택 영역 안에 변화된 픽셀이 없습니다.")
            self.line = []; self.is_drawing = False; self.update(); return
        self.bigbox.push_undo()
        added_count = 0
        for cnt in contours:
            if cv2.contourArea(cnt) < 5: continue
            pts_cv = cnt.reshape(-1, 2)
            flat = pts_cv.flatten().astype(float).tolist()
            if len(flat) >=6: self.poly_list.append({'points': flat, 'class': self.current_class}); added_count += 1
        if added_count > 0:
            self.bigbox.image_labels[self.path] = self.poly_list.copy(); self.bigbox.set_list_models()
        else: self.bigbox.undo_stack.pop(); QMessageBox.information(self, "알림", "선택 영역 안에 유효한 크기의 변화된 픽셀이 없습니다.")
        self.line = []; self.is_drawing = False; self.update()
        if hasattr(self, 'pair_box') and self.pair_box: self.pair_box.update()

    def keyPressEvent(self, event):
        # ... (기존 코드와 동일하게 유지)
        if event.key() == Qt.Key_A:
            if self.bigbox and (self.bigbox.auto_polygon_mode or \
                               (self.bigbox.region_select_mode and self.bigbox.current_region_selection_tool_type != "polygon")):
                return 
            if not self.is_drawing: self.is_drawing = True; self.line = []; self.line_redo_stack.clear()
            pos = self.mapFromGlobal(QCursor.pos()); self.is_moving = False
            synthetic_event = QMouseEvent(QEvent.MouseButtonRelease, pos, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
            self.mouseReleaseEvent(synthetic_event)
        elif event.key() == Qt.Key_Z and event.modifiers() & Qt.ControlModifier:
            if self.is_drawing:
                if self.line and len(self.line) >= 2:
                    self.line_redo_stack.append(self.line[-2:]); self.line = self.line[:-2]
                    self.is_closed = False; self.pos = self.mapFromGlobal(QCursor.pos()); self.update()
            elif self.bigbox: self.bigbox.undo()
        elif event.key() == Qt.Key_Y and event.modifiers() & Qt.ControlModifier:
            if self.is_drawing:
                if self.line_redo_stack:
                    self.line.extend(self.line_redo_stack.pop())
                    self.is_closed = False; self.pos = self.mapFromGlobal(QCursor.pos()); self.update()
            elif self.bigbox: self.bigbox.redo()
        else: super().keyPressEvent(event)


    def wheelEvent(self, event):
        # ... (기존 코드와 동일하게 유지, repaint() -> update() 변경)
        try:
            if not self.img or self.img.isNull(): return
            angleY = event.angleDelta().y() / 8 
            zoom_factor = 1.1; old_scale = self.scale
            self.scale *= zoom_factor if angleY > 0 else (1/zoom_factor)
            self.scale = max(0.1, min(self.scale, 10.0))
            new_w = self.img.width() * self.scale; new_h = self.img.height() * self.scale
            cursor_pos_widget = event.pos()
            img_x_at_cursor = (cursor_pos_widget.x() - self.point.x()) / old_scale
            img_y_at_cursor = (cursor_pos_widget.y() - self.point.y()) / old_scale
            new_point_x = cursor_pos_widget.x() - (img_x_at_cursor * self.scale)
            new_point_y = cursor_pos_widget.y() - (img_y_at_cursor * self.scale)
            self.point = QPoint(int(new_point_x), int(new_point_y))
            self.w = int(new_w); self.h = int(new_h)
            self.setFixedSize(self.w, self.h); self.update()
            if self.pair_box and self.pair_box.img and not self.pair_box.img.isNull():
                self.pair_box.scale = self.scale
                self.pair_box.point = QPoint(int(new_point_x), int(new_point_y))
                self.pair_box.w = int(self.pair_box.img.width() * self.pair_box.scale)
                self.pair_box.h = int(self.pair_box.img.height() * self.pair_box.scale)
                self.pair_box.setFixedSize(self.pair_box.w, self.pair_box.h); self.pair_box.update()
        except Exception as e: print(f"wheelEvent 오류: {e}")


class SynchronizedScrollArea(QScrollArea):
    # ... (기존 코드와 동일하게 유지)
    def __init__(self, parent=None):
        super(SynchronizedScrollArea, self).__init__(parent)
        self.pair_scroll = None; self.isScrolling = False
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn); self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.setAlignment(Qt.AlignCenter); self.setWidgetResizable(False)
    def set_pair_scroll(self, scroll_area): self.pair_scroll = scroll_area
    def scrollContentsBy(self, dx, dy):
        super().scrollContentsBy(dx, dy)
        if self.pair_scroll and not self.isScrolling:
            self.pair_scroll.isScrolling = True
            self.pair_scroll.horizontalScrollBar().setValue(self.horizontalScrollBar().value())
            self.pair_scroll.verticalScrollBar().setValue(self.verticalScrollBar().value())
            self.pair_scroll.isScrolling = False
    def wheelEvent(self, event):
        widget = self.widget()
        if widget and isinstance(widget, ImageBox) and (QApplication.keyboardModifiers() & Qt.ShiftModifier):
            widget.wheelEvent(event); event.accept(); return
        super().wheelEvent(event)
        if self.pair_scroll and not self.isScrolling:
            self.pair_scroll.isScrolling = True
            self.pair_scroll.horizontalScrollBar().setValue(self.horizontalScrollBar().value())
            self.pair_scroll.verticalScrollBar().setValue(self.verticalScrollBar().value())
            self.pair_scroll.isScrolling = False
        event.accept()


class change_detection(QMainWindow):
    def __init__(self, parent=None):
        super(change_detection, self).__init__(parent)
        # ... (멤버 변수 초기화는 이전과 동일)
        self.temp_listA = []; self.temp_listB = []; self.image_labels = {}
        self.undo_stack = []; self.redo_stack = []
        self.selected_a_image_path = None; self.selected_b_image_path = None
        self.change_detection_active = False; self.change_mask = None
        self.change_threshold = 30; self.original_before_img = None; self.original_after_img = None
        self.auto_polygon_mode = False
        
        # 영역 선택 모드 기본값 ON, 도구 기본값 'circle'
        self.region_select_mode = True 
        self.current_region_selection_tool_type = "circle" 
        self.current_shape_cursor_size = 50

        self.resize(int(1400 * 0.8), int(1100 * 0.8))
        self.click_timer = QTimer(); self.click_timer.setSingleShot(True)
        self.click_timer.timeout.connect(self.handle_single_click_label_list); self.click_index_label_list = None
        
        # --- UI 생성 및 설정 (기존과 대부분 동일, 영역 선택 도구 부분만 변경) ---
        importAct = QAction('가져오기', self, triggered=self.open_folder_dialog)
        saveAct = QAction('라벨 저장', self, triggered=self.savepoint, shortcut='Ctrl+S')
        undoAct = QAction('실행 취소', self, triggered=self.undo, shortcut='Ctrl+Z')
        redoAct = QAction('다시 실행', self, triggered=self.redo, shortcut='Ctrl+Y')
        exitAct = QAction('종료', self, shortcut='Ctrl+Q', triggered=self.close)
        bar = self.menuBar(); file_menu = bar.addMenu("파일"); file_menu.addActions([importAct, saveAct, undoAct, redoAct, exitAct])
        url_act = QAction("도움말 (GitHub)", self, triggered=lambda: webbrowser.open("https://github.com/qkrwogus98/CD_label"))
        help_menu = bar.addMenu("도움말"); help_menu.addAction(url_act)

        main_layout = QHBoxLayout(); self.comparison_widget = QWidget()
        comparison_layout = QHBoxLayout(self.comparison_widget); comparison_layout.setContentsMargins(0,0,0,0)
        self.before_scroll = SynchronizedScrollArea(); self.before_box = ImageBox(is_after=False)
        self.before_box.setMouseTracking(True); self.before_box.bigbox = self; self.before_scroll.setWidget(self.before_box)
        self.after_scroll = SynchronizedScrollArea(); self.after_box = ImageBox(is_after=True)
        self.after_box.setMouseTracking(True); self.after_box.bigbox = self; self.after_scroll.setWidget(self.after_box)
        self.before_scroll.set_pair_scroll(self.after_scroll); self.after_scroll.set_pair_scroll(self.before_scroll)
        self.before_box.set_pair_box(self.after_box); self.after_box.set_pair_box(self.before_box)
        self.after_box.poly_list = self.before_box.poly_list
        comparison_layout.addWidgets(self.before_scroll, self.after_scroll)

        right_panel_widget = QWidget(); V_RightPanel = QVBoxLayout(right_panel_widget)
        file_list_layout = QHBoxLayout(); V_A_Layout = QVBoxLayout(); V_B_Layout = QVBoxLayout()
        self.label_a = QLabel("변화 전(Before) 파일 목록"); self.import_btn = QPushButton("폴더 선택...")
        self.import_btn.clicked.connect(lambda: self.open_folder_dialog("A")); self.LV_A = QListView()
        self.LV_A.clicked.connect(self.on_file_list_selected); V_A_Layout.addWidgets(self.label_a, self.import_btn, self.LV_A)
        self.label_b = QLabel("변화 후(After) 파일 목록"); self.import_btnB = QPushButton("폴더 선택...")
        self.import_btnB.clicked.connect(lambda: self.open_folder_dialog("B")); self.LV_B = QListView()
        self.LV_B.clicked.connect(self.on_file_list_selected); V_B_Layout.addWidgets(self.label_b, self.import_btnB, self.LV_B)
        file_list_layout.addLayout(V_A_Layout); file_list_layout.addLayout(V_B_Layout); V_RightPanel.addLayout(file_list_layout)

        tool_box_widget = QWidget(); V_Tool = QVBoxLayout(tool_box_widget)
        save_buttons_layout = QHBoxLayout(); self.save_btn = QPushButton("라벨 저장 (Ctrl+S)"); self.save_btn.clicked.connect(self.savepoint)
        self.no_change_save_btn = QPushButton("변화 없음으로 저장"); self.no_change_save_btn.clicked.connect(self.save_no_change)
        save_buttons_layout.addWidgets(self.save_btn, self.no_change_save_btn); V_Tool.addLayout(save_buttons_layout)

        class_buttons_group = QWidget(); H_ClassButtons = QGridLayout(class_buttons_group); self.class_buttons = {}
        class_names = {1: "사람", 2: "돌", 3: "흙", 4: "물", 5: "불", 6: "나무"}
        for r, (class_num, class_name) in enumerate(class_names.items()): # 간결하게 수정
            btn = QPushButton(f"{class_name} ({class_num})")
            btn.clicked.connect(lambda checked, num=class_num: self.update_class_from_button(num))
            self.class_buttons[class_num] = btn; H_ClassButtons.addWidget(btn, r // 3, r % 3)
        V_Tool.addWidget(class_buttons_group)
        
        class_input_layout = QHBoxLayout(); self.class_input_label = QLabel('현재 선택 클래스:'); self.class_input = QLineEdit('1')
        self.class_input.setValidator(QIntValidator(1,6)); self.class_input.returnPressed.connect(self.update_class_from_input)
        self.class_input.textChanged.connect(self.highlight_class_button); class_input_layout.addWidgets(self.class_input_label, self.class_input)
        V_Tool.addLayout(class_input_layout); self.highlight_class_button(self.class_input.text())

        change_detection_label = QLabel("픽셀 변화 감지 도구"); change_detection_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        V_Tool.addWidget(change_detection_label); threshold_layout = QHBoxLayout(); threshold_label = QLabel("감지 민감도 (낮을수록 민감):")
        self.threshold_slider = QSlider(Qt.Horizontal); self.threshold_slider.setRange(1,100); self.threshold_slider.setValue(self.change_threshold)
        self.threshold_slider.setToolTip("값이 낮을수록 작은 변화도 감지합니다 (1~100)."); self.threshold_value_label = QLabel(f"{self.change_threshold}")
        self.threshold_slider.valueChanged.connect(self.update_threshold_value_label)
        threshold_layout.addWidgets(threshold_label, self.threshold_slider, self.threshold_value_label); V_Tool.addLayout(threshold_layout)
        change_detection_btns_layout = QHBoxLayout(); self.detect_changes_btn = QPushButton("픽셀 변화 감지 실행"); self.detect_changes_btn.clicked.connect(self.detect_pixel_changes)
        self.reset_images_btn = QPushButton("원본 이미지로 복원"); self.reset_images_btn.clicked.connect(self.reset_images_to_original)
        change_detection_btns_layout.addWidgets(self.detect_changes_btn, self.reset_images_btn); V_Tool.addLayout(change_detection_btns_layout)

        self.auto_polygon_btn = QPushButton("자동 폴리곤 모드 (OFF)"); self.auto_polygon_btn.setCheckable(True)
        self.auto_polygon_btn.clicked.connect(self.toggle_auto_polygon_mode); V_Tool.addWidget(self.auto_polygon_btn)
        self.region_select_btn = QPushButton("영역선택 모드 (ON)"); self.region_select_btn.setCheckable(True) # 기본 ON
        self.region_select_btn.setChecked(self.region_select_mode) # 초기 상태 반영
        self.region_select_btn.clicked.connect(self.toggle_region_select_mode); V_Tool.addWidget(self.region_select_btn)

        self.region_tool_group = QGroupBox("영역 선택 도구 옵션"); region_tool_group_layout = QVBoxLayout()
        self.radio_poly_draw = QRadioButton("폴리곤 직접 그리기"); self.radio_poly_draw.toggled.connect(self._update_region_selection_tool_type)
        self.radio_square_select = QRadioButton("정사각형으로 선택"); self.radio_square_select.toggled.connect(self._update_region_selection_tool_type)
        self.radio_circle_select = QRadioButton("원형으로 선택"); self.radio_circle_select.setChecked(True) # 기본 원형
        self.radio_circle_select.toggled.connect(self._update_region_selection_tool_type)
        region_tool_group_layout.addWidgets(self.radio_poly_draw, self.radio_square_select, self.radio_circle_select)
        
        shape_size_layout = QHBoxLayout(); shape_size_label_text = QLabel("도형 크기:")
        self.shape_size_slider = QSlider(Qt.Horizontal); self.shape_size_slider.setRange(20, 300); self.shape_size_slider.setValue(self.current_shape_cursor_size)
        self.shape_size_slider.valueChanged.connect(self._update_shape_cursor_size_from_slider)
        self.shape_size_value_label = QLabel(str(self.current_shape_cursor_size))
        shape_size_layout.addWidgets(shape_size_label_text, self.shape_size_slider, self.shape_size_value_label)
        region_tool_group_layout.addLayout(shape_size_layout)
        self.region_tool_group.setLayout(region_tool_group_layout); V_Tool.addWidget(self.region_tool_group)
        self.region_tool_group.setEnabled(self.region_select_mode) # 초기 활성화 상태

        # 스타일 및 툴팁 (기존과 동일)
        self.detect_changes_btn.setStyleSheet("QPushButton { background-color: #a970ff; color: white; font-weight: bold; padding: 5px; border-radius: 3px; } QPushButton:hover { background-color: #8a50e8; }")
        self.reset_images_btn.setStyleSheet("QPushButton { background-color: #555555; color: white; padding: 5px; border-radius: 3px; } QPushButton:hover { background-color: #777777; }")
        self.auto_polygon_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FF9800; } QPushButton:hover { background-color: #45a049; }")
        self.region_select_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FFC107; color: black; } QPushButton:hover { background-color: #1e88e5; }")
        self.detect_changes_btn.setToolTip("두 이미지 간의 픽셀 차이를 보라색으로 표시합니다.")
        self.reset_images_btn.setToolTip("변화 감지 모드를 해제하고 원래 이미지 상태로 돌아갑니다.")
        self.auto_polygon_btn.setToolTip("활성화 시, 변화 감지된 영역을 클릭하여 자동으로 폴리곤을 생성합니다.")
        self.region_select_btn.setToolTip("활성화 시, 지정한 영역 내의 변화를 폴리곤으로 자동 생성합니다.")
        self.region_tool_group.setToolTip("영역 선택 방식을 선택하고 도형 크기를 조절합니다.")
        for btn_h in [self.detect_changes_btn, self.reset_images_btn, self.auto_polygon_btn, self.region_select_btn]: btn_h.setMinimumHeight(30)
        
        V_RightPanel.addWidget(tool_box_widget)
        label_list_label = QLabel("현재 이미지 라벨 목록 (더블클릭: 수정, 우클릭: 삭제)"); V_RightPanel.addWidget(label_list_label)
        self.LV_label = QListView(); self.LV_label.setContextMenuPolicy(Qt.CustomContextMenu)
        self.LV_label.customContextMenuRequested.connect(self.show_label_list_context_menu)
        self.LV_label.clicked.connect(self.start_single_click_timer_label_list)
        self.LV_label.doubleClicked.connect(self.handle_double_click_label_list); V_RightPanel.addWidget(self.LV_label)

        self.auto_adding_points = False; self.auto_add_interval = 200; self.auto_add_timer = QTimer()
        self.auto_add_timer.timeout.connect(self.add_point_at_cursor_auto)

        H_Splitter = QSplitter(Qt.Horizontal); H_Splitter.addWidget(self.comparison_widget); H_Splitter.addWidget(right_panel_widget)
        H_Splitter.setStretchFactor(0, 3); H_Splitter.setStretchFactor(1, 1); H_Splitter.setSizes([int(self.width()*0.7), int(self.width()*0.3)])
        main_layout.addWidget(H_Splitter); main_frame = QWidget(); main_frame.setLayout(main_layout); self.setCentralWidget(main_frame)
        self.setWindowTitle("변화 감지 라벨링 도구 (Ver 0.2.3 - 우클릭 이동 및 기본값 변경)"); self.setFocusPolicy(Qt.StrongFocus)
        
        # 초기 상태 업데이트 호출
        self.toggle_region_select_mode() # 버튼 상태에 따라 UI 업데이트
        self._update_region_selection_tool_type() # 라디오 버튼 상태에 따라 UI 업데이트 및 ImageBox에 전달

        self.showMaximized()
        self.set_list_models()


    def _update_region_selection_tool_type(self):
        # ... (기존 코드와 동일)
        if self.radio_poly_draw.isChecked():
            self.current_region_selection_tool_type = "polygon"; self.shape_size_slider.setEnabled(False)
        elif self.radio_square_select.isChecked():
            self.current_region_selection_tool_type = "square"; self.shape_size_slider.setEnabled(self.region_select_mode)
        elif self.radio_circle_select.isChecked():
            self.current_region_selection_tool_type = "circle"; self.shape_size_slider.setEnabled(self.region_select_mode)
        self.before_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, self.region_select_mode)
        self.after_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, self.region_select_mode)
        print(f"영역 선택 도구 변경: {self.current_region_selection_tool_type}, 크기 슬라이더 활성: {self.shape_size_slider.isEnabled()}")


    def _update_shape_cursor_size_from_slider(self, value):
        # ... (기존 코드와 동일)
        self.current_shape_cursor_size = value; self.shape_size_value_label.setText(str(value))
        self.before_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, self.region_select_mode)
        self.after_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, self.region_select_mode)

    def toggle_region_select_mode(self):
        # ... (기존 코드와 동일, 초기 호출 시 self.region_select_btn.isChecked()가 정확한 상태 반영)
        self.region_select_mode = self.region_select_btn.isChecked()
        self.region_tool_group.setEnabled(self.region_select_mode)
        if self.region_select_mode:
            self.region_select_btn.setText("영역선택 모드 (ON)")
            self.region_select_btn.setStyleSheet("QPushButton:checked { background-color: #FFC107; color: black; padding: 5px; border-radius: 3px; } QPushButton { background-color: #2196F3; color: white; padding: 5px; border-radius: 3px; }")
            if self.auto_polygon_mode:
                self.auto_polygon_btn.setChecked(False); self.auto_polygon_mode = False
                self.auto_polygon_btn.setText("자동 폴리곤 모드 (OFF)")
                self.auto_polygon_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FF9800; } QPushButton:hover { background-color: #45a049; }")
            self._update_region_selection_tool_type() 
        else:
            self.region_select_btn.setText("영역선택 모드 (OFF)")
            self.region_select_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FFC107; color: black; } QPushButton:hover { background-color: #1e88e5; }")
            self.before_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, False)
            self.after_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, False)
        print(f"영역선택 모드: {'ON' if self.region_select_mode else 'OFF'}")

    def toggle_auto_polygon_mode(self):
        # ... (기존 코드와 동일)
        self.auto_polygon_mode = self.auto_polygon_btn.isChecked()
        if self.auto_polygon_mode:
            self.auto_polygon_btn.setText("자동 폴리곤 모드 (ON)")
            self.auto_polygon_btn.setStyleSheet("QPushButton:checked { background-color: #FF9800; color: white; padding: 5px; border-radius: 3px; } QPushButton { background-color: #FF9800; color: white; padding: 5px; border-radius: 3px; }")
            if not self.change_detection_active or self.change_mask is None:
                QMessageBox.information(self, "알림", "자동 폴리곤 모드를 사용하려면 먼저 '픽셀 변화 감지 실행'을 해주세요.")
                self.auto_polygon_btn.setChecked(False); self.auto_polygon_mode = False
                self.auto_polygon_btn.setText("자동 폴리곤 모드 (OFF)")
                self.auto_polygon_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FF9800; } QPushButton:hover { background-color: #45a049; }")
            if self.region_select_mode:
                self.region_select_btn.setChecked(False); self.region_select_mode = False
                self.region_select_btn.setText("영역선택 모드 (OFF)")
                self.region_select_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FFC107; color: black; } QPushButton:hover { background-color: #1e88e5; }")
                self.region_tool_group.setEnabled(False)
                self.before_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, False)
                self.after_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, False)
        else:
            self.auto_polygon_btn.setText("자동 폴리곤 모드 (OFF)")
            self.auto_polygon_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px; border-radius: 3px; } QPushButton:checked { background-color: #FF9800; } QPushButton:hover { background-color: #45a049; }")
        print(f"자동 폴리곤 모드: {'ON' if self.auto_polygon_mode else 'OFF'}")

    # --- 나머지 메소드들은 이전 버전과 동일하게 유지 (savepoint, on_file_list_selected 등) ---
    # ... (생략된 나머지 메소드들은 이전 제공 코드와 동일합니다)
    def update_threshold_value_label(self, value):
        self.change_threshold = value
        self.threshold_value_label.setText(f"{value}")

    def detect_pixel_changes(self):
        try:
            if not self.selected_a_image_path or not self.selected_b_image_path:
                QMessageBox.information(self, "알림", "변화 전/후 이미지가 모두 선택되지 않았습니다.")
                return
            if self.original_before_img is None and self.before_box.img:
                self.original_before_img = self.before_box.img.copy()
            if self.original_after_img is None and self.after_box.img:
                self.original_after_img = self.after_box.img.copy()

            img_array_a = np.fromfile(self.selected_a_image_path, np.uint8)
            img_a_cv = cv2.imdecode(img_array_a, cv2.IMREAD_COLOR)
            img_array_b = np.fromfile(self.selected_b_image_path, np.uint8)
            img_b_cv = cv2.imdecode(img_array_b, cv2.IMREAD_COLOR)

            if img_a_cv is None or img_b_cv is None:
                QMessageBox.warning(self, "오류", "이미지 로드에 실패했습니다.")
                return
            if img_a_cv.shape != img_b_cv.shape:
                img_b_cv = cv2.resize(img_b_cv, (img_a_cv.shape[1], img_a_cv.shape[0]))

            diff = cv2.absdiff(img_a_cv, img_b_cv)
            mask = np.any(diff > self.change_threshold, axis=2).astype(np.uint8) * 255
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            self.change_mask = mask.copy()
            print("변화 감지 마스크 (self.change_mask) 생성됨.")

            img_b_with_changes_cv = img_b_cv.copy()
            img_b_with_changes_cv[mask > 0] = (180, 0, 180) # 보라색

            def cv_to_qpixmap(cv_img):
                rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                return QPixmap.fromImage(qt_image)

            result_pixmap = cv_to_qpixmap(img_b_with_changes_cv)
            self.before_box.img = result_pixmap
            self.after_box.img = result_pixmap
            h, w = img_b_cv.shape[:2]
            self.before_box.w = int(w * self.before_box.scale)
            self.before_box.h = int(h * self.before_box.scale)
            self.before_box.setFixedSize(self.before_box.w, self.before_box.h)
            self.after_box.w = int(w * self.after_box.scale)
            self.after_box.h = int(h * self.after_box.scale)
            self.after_box.setFixedSize(self.after_box.w, self.after_box.h)
            self.change_detection_active = True
            self.before_box.update()
            self.after_box.update()
            QMessageBox.information(self, "픽셀 변화 감지 완료", f"변화가 보라색으로 표시되었습니다 (민감도: {self.change_threshold}).")
        except Exception as e:
            print(f"detect_pixel_changes 오류: {e}")
            QMessageBox.warning(self, "오류", f"픽셀 변화 감지 중 오류 발생: {e}")
            self.change_detection_active = False
            self.change_mask = None

    def reset_images_to_original(self):
        try:
            if self.change_detection_active:
                if self.original_before_img and not self.original_before_img.isNull():
                    self.before_box.img = self.original_before_img.copy()
                elif self.selected_a_image_path:
                    self.before_box.path = self.selected_a_image_path
                    self.before_box.set_image()
                else:
                    self.before_box.img = QPixmap(); self.before_box.setFixedSize(400,400)

                if self.original_after_img and not self.original_after_img.isNull():
                    self.after_box.img = self.original_after_img.copy()
                elif self.selected_b_image_path:
                    self.after_box.path = self.selected_b_image_path
                    self.after_box.set_image()
                else:
                    self.after_box.img = QPixmap(); self.after_box.setFixedSize(400,400)

                if self.before_box.img and not self.before_box.img.isNull():
                    self.before_box.w = int(self.before_box.img.width() * self.before_box.scale)
                    self.before_box.h = int(self.before_box.img.height() * self.before_box.scale)
                    self.before_box.setFixedSize(self.before_box.w, self.before_box.h)
                if self.after_box.img and not self.after_box.img.isNull():
                    self.after_box.w = int(self.after_box.img.width() * self.after_box.scale)
                    self.after_box.h = int(self.after_box.img.height() * self.after_box.scale)
                    self.after_box.setFixedSize(self.after_box.w, self.after_box.h)
                
                self.original_before_img = None
                self.original_after_img = None
                self.change_detection_active = False
                self.change_mask = None
                if self.auto_polygon_mode:
                    self.auto_polygon_btn.setChecked(False); self.toggle_auto_polygon_mode()
                self.before_box.update(); self.after_box.update()
                QMessageBox.information(self, "원본 복원", "원본 이미지로 복원되었습니다.")
            else:
                if self.selected_a_image_path: self.before_box.path = self.selected_a_image_path; self.before_box.set_image()
                if self.selected_b_image_path: self.after_box.path = self.selected_b_image_path; self.after_box.set_image()
                QMessageBox.information(self, "알림", "이미 원본 상태이거나 변화 감지가 실행되지 않았습니다.")
        except Exception as e:
            print(f"reset_images_to_original 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 초기화 실패: {e}")

    def save_no_change(self):
        try:
            if not self.selected_a_image_path or not self.selected_b_image_path:
                QMessageBox.information(self, "알림", "저장할 이미지가 선택되지 않았습니다.")
                return
            current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
            binary_dir = os.path.join(current_dir, 'binary_cd')
            semantic_dir = os.path.join(current_dir, 'semantic_cd')
            label_stat_dir = os.path.join(current_dir, "label_stats")
            os.makedirs(binary_dir, exist_ok=True)
            os.makedirs(semantic_dir, exist_ok=True)
            os.makedirs(label_stat_dir, exist_ok=True)

            target_image_path = self.selected_b_image_path or self.selected_a_image_path
            if not target_image_path: return

            img_array = np.fromfile(target_image_path, np.uint8)
            img_cv = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if img_cv is None:
                QMessageBox.warning(self, "오류", f"이미지 로드 실패: {target_image_path}"); return
            
            height, width = img_cv.shape[:2]
            black_image_cv = np.zeros((height, width, 3), dtype=np.uint8)
            base_filename = os.path.splitext(os.path.basename(target_image_path))[0]
            binary_filename = os.path.join(binary_dir, f"{base_filename}.jpg")
            semantic_filename = os.path.join(semantic_dir, f"{base_filename}.jpg")
            cv2.imwrite(binary_filename, black_image_cv)
            cv2.imwrite(semantic_filename, black_image_cv)
            append_no_change_file(label_stat_dir, self.selected_a_image_path, self.selected_b_image_path)
            QMessageBox.information(self, "저장 완료", f"'변화 없음' 라벨 저장 완료:\n{binary_filename}\n{semantic_filename}")
        except Exception as e:
            print(f"save_no_change 오류: {e}")
            QMessageBox.warning(self, "오류", f"변화 없음 라벨 저장 실패: {e}")

    def add_point_at_cursor_auto(self):
        if not self.auto_adding_points: self.auto_add_timer.stop(); return
        focused_widget = QApplication.focusWidget()
        if isinstance(focused_widget, ImageBox) and focused_widget.is_drawing:
            pos = focused_widget.mapFromGlobal(QCursor.pos())
            focused_widget.is_moving = False
            synthetic_event = QMouseEvent(QEvent.MouseButtonRelease, pos, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
            focused_widget.mouseReleaseEvent(synthetic_event)
        else: self.auto_adding_points = False; self.auto_add_timer.stop(); print("자동 포인트 추가 중지: 조건 미충족")

    def update_class_from_button(self, class_number):
        if 1 <= class_number <= 6:
            self.before_box.current_class = class_number
            self.after_box.current_class = class_number
            self.class_input.setText(str(class_number))

    def update_class_from_input(self):
        try:
            class_number_str = self.class_input.text()
            if not class_number_str.isdigit():
                QMessageBox.warning(self, "입력 오류", "클래스 번호는 숫자여야 합니다.")
                self.class_input.setText(str(self.before_box.current_class)); return
            class_number = int(class_number_str)
            if 1 <= class_number <= 6:
                self.before_box.current_class = class_number
                self.after_box.current_class = class_number
            else:
                QMessageBox.warning(self, "입력 오류", "클래스 번호는 1에서 6 사이여야 합니다.")
                self.class_input.setText(str(self.before_box.current_class))
        except ValueError:
            QMessageBox.warning(self, "입력 오류", "유효한 숫자를 입력해주세요.")
            self.class_input.setText(str(self.before_box.current_class))
        except Exception as e:
            print(f"update_class_from_input 오류: {e}")

    def highlight_class_button(self, class_text):
        try:
            current_class_num = int(class_text) if class_text.isdigit() else -1
            for num, btn in self.class_buttons.items():
                btn.setStyleSheet("background-color: lightblue; font-weight: bold;" if num == current_class_num else "")
        except ValueError:
            for btn in self.class_buttons.values(): btn.setStyleSheet("")

    def start_single_click_timer_label_list(self, index):
        self.click_index_label_list = index
        self.click_timer.start(QApplication.doubleClickInterval())

    def handle_single_click_label_list(self):
        if self.click_index_label_list and self.click_index_label_list.isValid():
            self.select_polygon_from_list(self.click_index_label_list, double_click=False)

    def handle_double_click_label_list(self, index):
        self.click_timer.stop()
        if index.isValid():
            self.select_polygon_from_list(index, double_click=True)

    def select_polygon_from_list(self, index, double_click=False):
        try:
            poly_idx_in_list = index.row()
            if not (0 <= poly_idx_in_list < len(self.before_box.poly_list)): return
            self.before_box.selected_poly_index = poly_idx_in_list
            self.after_box.selected_poly_index = poly_idx_in_list
            self.before_box.update(); self.after_box.update()
            if double_click:
                poly_dict_to_edit = self.before_box.poly_list[poly_idx_in_list]
                current_class_val = poly_dict_to_edit.get('class', self.before_box.current_class)
                new_class_num, ok = QInputDialog.getInt(self, "클래스 수정", "새 클래스 번호 (1~6):",
                                                     value=current_class_val, min=1, max=6)
                if ok:
                    self.push_undo()
                    self.before_box.poly_list[poly_idx_in_list]['class'] = new_class_num
                    if self.before_box.path:
                        self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()
                    self.before_box.update(); self.after_box.update()
                    self.set_list_models()
        except Exception as e:
            print(f"select_polygon_from_list 오류: {e}")

    def show_label_list_context_menu(self, point):
        selected_indexes = self.LV_label.selectedIndexes()
        if not selected_indexes: return
        rightMenu = QMenu(self.LV_label)
        removeAction = QAction("선택한 라벨 삭제", self, triggered=self.remove_selected_polygon_from_list)
        rightMenu.addAction(removeAction)
        rightMenu.exec_(self.LV_label.mapToGlobal(point))

    def remove_selected_polygon_from_list(self):
        try:
            selected_indexes = self.LV_label.selectedIndexes()
            if not selected_indexes: return
            rows_to_remove = sorted(list(set(index.row() for index in selected_indexes)), reverse=True)
            if not rows_to_remove: return
            self.push_undo()
            for row_idx in rows_to_remove:
                if 0 <= row_idx < len(self.before_box.poly_list):
                    self.before_box.poly_list.pop(row_idx)
            self.before_box.selected_poly_index = -1
            self.after_box.selected_poly_index = -1
            if self.before_box.path:
                self.image_labels[self.before_box.path] = self.before_box.poly_list.copy()
            self.before_box.update(); self.after_box.update()
            self.set_list_models()
        except Exception as e:
            print(f"remove_selected_polygon_from_list 오류: {e}")

    def push_undo(self, clear_redo=True):
        if self.before_box.path:
            current_polygons_copy = [p.copy() for p in self.before_box.poly_list]
            self.undo_stack.append({'path': self.before_box.path, 'polygons': current_polygons_copy})
            if clear_redo: self.redo_stack.clear()

    def undo(self):
        try:
            if not self.undo_stack: QMessageBox.information(self, "실행 취소", "되돌릴 작업이 없습니다."); return
            if self.before_box.path:
                current_polygons_copy = [p.copy() for p in self.before_box.poly_list]
                self.redo_stack.append({'path': self.before_box.path, 'polygons': current_polygons_copy})
            last_state = self.undo_stack.pop()
            restored_path = last_state['path']
            restored_polygons = last_state['polygons']
            if self.before_box.path == restored_path:
                self.before_box.poly_list = [p.copy() for p in restored_polygons]
                self.image_labels[restored_path] = self.before_box.poly_list.copy()
                self.before_box.selected_poly_index = -1; self.after_box.selected_poly_index = -1
                self.before_box.update(); self.after_box.update()
                self.set_list_models()
            else:
                self.redo_stack.pop(); self.undo_stack.append(last_state)
                QMessageBox.warning(self, "실행 취소 오류", "현재 작업 이미지와 다른 이미지의 상태로는 되돌릴 수 없습니다.")
        except Exception as e: print(f"undo 오류: {e}")

    def redo(self):
        try:
            if not self.redo_stack: QMessageBox.information(self, "다시 실행", "다시 실행할 작업이 없습니다."); return
            if self.before_box.path:
                current_polygons_copy = [p.copy() for p in self.before_box.poly_list]
                self.undo_stack.append({'path': self.before_box.path, 'polygons': current_polygons_copy})
            next_state = self.redo_stack.pop()
            restored_path = next_state['path']
            restored_polygons = next_state['polygons']
            if self.before_box.path == restored_path:
                self.before_box.poly_list = [p.copy() for p in restored_polygons]
                self.image_labels[restored_path] = self.before_box.poly_list.copy()
                self.before_box.selected_poly_index = -1; self.after_box.selected_poly_index = -1
                self.before_box.update(); self.after_box.update()
                self.set_list_models()
            else:
                self.undo_stack.pop(); self.redo_stack.append(next_state)
                QMessageBox.warning(self, "다시 실행 오류", "현재 작업 이미지와 다른 이미지의 상태로는 다시 실행할 수 없습니다.")
        except Exception as e: print(f"redo 오류: {e}")

    def on_file_list_selected(self, qModelIndex):
        try:
            sender_list_view = self.sender()
            if not isinstance(sender_list_view, QListView): return
            selected_row = qModelIndex.row()
            if self.before_box.path and self.before_box.poly_list:
                self.image_labels[self.before_box.path] = [p.copy() for p in self.before_box.poly_list]
            self.undo_stack.clear(); self.redo_stack.clear()

            if 0 <= selected_row < len(self.temp_listA) and 0 <= selected_row < len(self.temp_listB):
                new_a_path = self.temp_listA[selected_row]
                new_b_path = self.temp_listB[selected_row]
                if self.selected_a_image_path != new_a_path or self.selected_b_image_path != new_b_path:
                    self.selected_a_image_path = new_a_path
                    self.selected_b_image_path = new_b_path
                    self.before_box.path = self.selected_a_image_path; self.before_box.set_image()
                    self.after_box.path = self.selected_b_image_path; self.after_box.set_image()
                    if self.before_box.path in self.image_labels:
                        self.before_box.poly_list = [p.copy() for p in self.image_labels[self.before_box.path]]
                    else: self.before_box.poly_list = []
                    self.after_box.poly_list = self.before_box.poly_list
                    self.change_detection_active = False; self.change_mask = None
                    self.original_before_img = None; self.original_after_img = None
                    if self.auto_polygon_mode: self.auto_polygon_btn.setChecked(False); self.toggle_auto_polygon_mode()
                    if self.region_select_mode: 
                        # 이미지 변경 시 영역 선택 모드는 유지하되, ImageBox의 그리기 상태는 초기화
                        self.before_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, True)
                        self.after_box.set_region_tool_params(self.current_region_selection_tool_type, self.current_shape_cursor_size, True)
                    else: # 영역 선택 모드가 아니면 ImageBox에도 반영
                        self.before_box.set_region_tool_params(None, self.current_shape_cursor_size, False)
                        self.after_box.set_region_tool_params(None, self.current_shape_cursor_size, False)


                    self.before_box.selected_poly_index = -1; self.after_box.selected_poly_index = -1
                    self.before_box.line = []; self.before_box.is_drawing = False; self.before_box.is_closed = False
                    self.after_box.line = []; self.after_box.is_drawing = False; self.after_box.is_closed = False
                    self.before_box.update(); self.after_box.update()
                    self.set_list_models()
                    if self.selected_a_image_path and self.selected_b_image_path:
                        print("이미지 선택 완료, 자동 픽셀 변화 감지 실행...")
                        self.detect_pixel_changes()
                if sender_list_view == self.LV_A: self.LV_B.setCurrentIndex(self.LV_B.model().index(selected_row, 0))
                elif sender_list_view == self.LV_B: self.LV_A.setCurrentIndex(self.LV_A.model().index(selected_row, 0))
            else: print(f"선택한 파일 인덱스 {selected_row}가 유효하지 않습니다.")
        except Exception as e:
            print(f"on_file_list_selected 오류: {e}")
            QMessageBox.warning(self, "오류", f"이미지 쌍 로드 실패: {e}")

    def open_folder_dialog(self, list_type):
        try:
            if self.before_box.path and self.before_box.poly_list:
                self.image_labels[self.before_box.path] = [p.copy() for p in self.before_box.poly_list]
            file_path, _ = QFileDialog.getOpenFileName(self, f"{list_type} 이미지 파일 선택", "", "Images (*.png *.jpg *.jpeg *.bmp)")
            if not file_path: return
            file_path = os.path.abspath(file_path)
            folder_path = os.path.dirname(file_path)
            all_images = sorted([os.path.abspath(os.path.join(folder_path, f)) for f in os.listdir(folder_path) if f.lower().endswith((".png", ".jpg", ".jpeg", ".bmp"))], key=natural_key)
            try: start_idx = all_images.index(file_path)
            except ValueError: QMessageBox.warning(self, "오류", "선택한 파일을 폴더에서 찾을 수 없습니다."); return
            subset_images = all_images[start_idx:start_idx + 100]
            if list_type == "A": self.temp_listA = subset_images
            elif list_type == "B": self.temp_listB = subset_images
            self.set_list_models()
            if self.temp_listA and self.temp_listB and len(self.temp_listA) == len(self.temp_listB):
                first_index = self.LV_A.model().index(0, 0)
                if first_index.isValid(): self.LV_A.setCurrentIndex(first_index); self.on_file_list_selected(first_index)
            else:
                QMessageBox.warning(self, "경고", f"Before/After 이미지 수가 다릅니다.\nBefore: {len(self.temp_listA)}개, After: {len(self.temp_listB)}개")
        except Exception as e: print(f"open_folder_dialog 오류 ({list_type}): {e}")

    def savepoint(self):
        try:
            if self.before_box.path and self.before_box.poly_list:
                self.image_labels[self.before_box.path] = [p.copy() for p in self.before_box.poly_list]
            if not self.image_labels: QMessageBox.information(self, "알림", "저장할 라벨 데이터가 없습니다."); return
            current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
            binary_dir = os.path.join(current_dir, 'binary_cd'); os.makedirs(binary_dir, exist_ok=True)
            semantic_dir = os.path.join(current_dir, 'semantic_cd'); os.makedirs(semantic_dir, exist_ok=True)
            label_stat_dir = os.path.join(current_dir, "label_stats"); os.makedirs(label_stat_dir, exist_ok=True)
            saved_count = 0
            for image_path, poly_list_for_image in self.image_labels.items():
                if not poly_list_for_image: continue
                img_array = np.fromfile(image_path, np.uint8)
                original_cv_img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if original_cv_img is None: print(f"원본 이미지 로드 실패 (저장 건너뜀): {image_path}"); continue
                height, width = original_cv_img.shape[:2]
                binary_mask_cv = np.zeros((height, width), dtype=np.uint8)
                semantic_mask_cv = np.zeros((height, width, 3), dtype=np.uint8)
                for poly_dict in poly_list_for_image:
                    points_relative = poly_dict.get('points', [])
                    class_number = poly_dict.get('class', 0)
                    if len(points_relative) < 6: continue
                    polygon_points_cv = [[int(points_relative[i]), int(points_relative[i+1])] for i in range(0, len(points_relative), 2)]
                    if not polygon_points_cv: continue
                    np_poly_pts = np.array([polygon_points_cv], dtype=np.int32)
                    cv2.fillPoly(binary_mask_cv, np_poly_pts, 255)
                    color_bgr = {1:(0,255,255), 2:(128,128,128), 3:(19,69,139), 4:(255,0,0), 5:(0,0,255), 6:(0,128,0)}.get(class_number, (0,0,0))
                    cv2.fillPoly(semantic_mask_cv, np_poly_pts, color_bgr)
                base_filename = os.path.splitext(os.path.basename(image_path))[0]
                binary_save_path = os.path.join(binary_dir, f"{base_filename}.png")
                semantic_save_path = os.path.join(semantic_dir, f"{base_filename}.png")
                cv2.imwrite(binary_save_path, binary_mask_cv)
                cv2.imwrite(semantic_save_path, semantic_mask_cv)
                saved_count +=1
            if saved_count > 0:
                QMessageBox.information(self, "저장 완료", f"{saved_count}개 이미지 라벨 마스크 저장 완료.")
                save_label_statistics(self.image_labels, label_stat_dir)
            else: QMessageBox.information(self, "알림", "저장할 유효 라벨 이미지가 없습니다.")
            self.set_list_models()
        except Exception as e:
            print(f"savepoint 오류: {e}\n{traceback.format_exc()}")
            QMessageBox.critical(self, "저장 오류", f"라벨 저장 중 오류: {e}")

    def set_list_models(self):
        try:
            model_a = QStandardItemModel(); model_b = QStandardItemModel()
            # temp_listA와 temp_listB의 길이를 맞춰서 처리 (짧은 쪽 기준)
            len_a = len(self.temp_listA)
            len_b = len(self.temp_listB)
            common_len = min(len_a, len_b)

            for i in range(common_len):
                path_a = self.temp_listA[i]
                item_a = QStandardItem(os.path.basename(path_a))
                if path_a == self.selected_a_image_path: font = item_a.font(); font.setBold(True); item_a.setFont(font)
                model_a.appendRow(item_a)

                path_b = self.temp_listB[i]
                item_b = QStandardItem(os.path.basename(path_b))
                if path_b == self.selected_b_image_path: font = item_b.font(); font.setBold(True); item_b.setFont(font)
                model_b.appendRow(item_b)
            
            # 남은 파일들 처리
            if len_a > common_len:
                for i in range(common_len, len_a):
                    item_a = QStandardItem(os.path.basename(self.temp_listA[i]))
                    if self.temp_listA[i] == self.selected_a_image_path: font = item_a.font(); font.setBold(True); item_a.setFont(font)
                    model_a.appendRow(item_a)
            elif len_b > common_len:
                 for i in range(common_len, len_b):
                    item_b = QStandardItem(os.path.basename(self.temp_listB[i]))
                    if self.temp_listB[i] == self.selected_b_image_path: font = item_b.font(); font.setBold(True); item_b.setFont(font)
                    model_b.appendRow(item_b)

            self.LV_A.setModel(model_a); self.LV_B.setModel(model_b)
            
            poly_info = []
            class_map = {1:"사람", 2:"돌", 3:"흙", 4:"물", 5:"불", 6:"나무"}
            for idx, poly_dict in enumerate(self.before_box.poly_list): # before_box 기준으로 poly_list 접근
                cls_name = class_map.get(poly_dict.get('class',0), f"미지정({poly_dict.get('class',0)})")
                poly_info.append(f"폴리곤 {idx+1}: {cls_name}")
            self.LV_label.setModel(QStringListModel(poly_info))
        except Exception as e: print(f"set_list_models 오류: {e}")


    def closeEvent(self, event):
        reply = QMessageBox.question(self, "프로그램 종료", "변경사항을 저장하지 않고 종료하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            if self.auto_adding_points: self.auto_add_timer.stop()
            event.accept()
        else: event.ignore()

def addWidgets(layout, *widgets):
    for widget in widgets:
        layout.addWidget(widget)
QVBoxLayout.addWidgets = addWidgets
QHBoxLayout.addWidgets = addWidgets


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = change_detection()
    # win.show() # showMaximized()가 있으므로 중복 호출 필요 없음
    win.setFocus()
    sys.exit(app.exec_())