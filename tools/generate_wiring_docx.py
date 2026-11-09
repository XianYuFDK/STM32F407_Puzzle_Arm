from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


OUT = Path(r"D:\STM32\HAL\26\STM32F407VGT6机械臂工程接线表.docx")

BLUE = "2E74B5"
DARK_BLUE = "1F4D78"
INK = "0B2545"
MUTED = "606A78"
HEADER_FILL = "E8EEF5"
LIGHT_FILL = "F4F6F9"
WARNING_FILL = "FFF4CE"
BORDER = "AAB6C4"
WHITE = "FFFFFF"


def set_font(run, size=None, bold=None, color=None, latin="Calibri", east="Microsoft YaHei"):
    run.font.name = latin
    run._element.get_or_add_rPr().rFonts.set(qn("w:ascii"), latin)
    run._element.get_or_add_rPr().rFonts.set(qn("w:hAnsi"), latin)
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), east)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if color is not None:
        run.font.color.rgb = RGBColor.from_string(color)


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=80, start=120, bottom=80, end=120):
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for edge, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        tag = "w:" + edge
        node = tc_mar.find(qn(tag))
        if node is None:
            node = OxmlElement(tag)
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_repeat_table_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    tbl_header = OxmlElement("w:tblHeader")
    tbl_header.set(qn("w:val"), "true")
    tr_pr.append(tbl_header)


def set_row_keep(row):
    tr_pr = row._tr.get_or_add_trPr()
    cant_split = OxmlElement("w:cantSplit")
    tr_pr.append(cant_split)


def set_table_geometry(table, widths):
    total = sum(widths)
    table.autofit = False
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    tbl_pr = table._tbl.tblPr

    tbl_w = tbl_pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        tbl_pr.append(tbl_w)
    tbl_w.set(qn("w:w"), str(total))
    tbl_w.set(qn("w:type"), "dxa")

    tbl_ind = tbl_pr.find(qn("w:tblInd"))
    if tbl_ind is None:
        tbl_ind = OxmlElement("w:tblInd")
        tbl_pr.append(tbl_ind)
    tbl_ind.set(qn("w:w"), "120")
    tbl_ind.set(qn("w:type"), "dxa")

    layout = tbl_pr.find(qn("w:tblLayout"))
    if layout is None:
        layout = OxmlElement("w:tblLayout")
        tbl_pr.append(layout)
    layout.set(qn("w:type"), "fixed")

    borders = tbl_pr.find(qn("w:tblBorders"))
    if borders is None:
        borders = OxmlElement("w:tblBorders")
        tbl_pr.append(borders)
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        node = borders.find(qn("w:" + edge))
        if node is None:
            node = OxmlElement("w:" + edge)
            borders.append(node)
        node.set(qn("w:val"), "single")
        node.set(qn("w:sz"), "4")
        node.set(qn("w:color"), BORDER)

    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths:
        col = OxmlElement("w:gridCol")
        col.set(qn("w:w"), str(width))
        grid.append(col)

    for row in table.rows:
        for index, cell in enumerate(row.cells):
            width = widths[index]
            tc_pr = cell._tc.get_or_add_tcPr()
            tc_w = tc_pr.find(qn("w:tcW"))
            if tc_w is None:
                tc_w = OxmlElement("w:tcW")
                tc_pr.append(tc_w)
            tc_w.set(qn("w:w"), str(width))
            tc_w.set(qn("w:type"), "dxa")
            set_cell_margins(cell)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER


def set_paragraph_border_bottom(paragraph, color=BLUE, size="12", space="6"):
    p_pr = paragraph._p.get_or_add_pPr()
    p_bdr = p_pr.find(qn("w:pBdr"))
    if p_bdr is None:
        p_bdr = OxmlElement("w:pBdr")
        p_pr.append(p_bdr)
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), size)
    bottom.set(qn("w:space"), space)
    bottom.set(qn("w:color"), color)
    p_bdr.append(bottom)


def add_field(paragraph, instruction):
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = instruction
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    text = OxmlElement("w:t")
    text.text = "1"
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run = paragraph.add_run()
    run._r.extend([begin, instr, separate, text, end])
    set_font(run, size=9, color=MUTED)


def add_heading(doc, text, level=1):
    p = doc.add_paragraph(style=f"Heading {level}")
    p.paragraph_format.keep_with_next = True
    run = p.add_run(text)
    return p


def add_note_box(doc, title, body, warning=False):
    table = doc.add_table(rows=1, cols=1)
    set_table_geometry(table, [9360])
    cell = table.cell(0, 0)
    set_cell_shading(cell, WARNING_FILL if warning else LIGHT_FILL)
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(2)
    r = p.add_run(title)
    set_font(r, size=10.5, bold=True, color="7A5A00" if warning else DARK_BLUE)
    p2 = cell.add_paragraph()
    p2.paragraph_format.space_after = Pt(0)
    p2.paragraph_format.line_spacing = 1.2
    r2 = p2.add_run(body)
    set_font(r2, size=10.5)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)


def add_table(doc, headers, rows, widths, center_columns=()):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    hdr = table.rows[0]
    set_repeat_table_header(hdr)
    set_row_keep(hdr)
    for i, value in enumerate(headers):
        cell = hdr.cells[i]
        cell.text = str(value)
        set_cell_shading(cell, HEADER_FILL)
        p = cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_after = Pt(0)
        for run in p.runs:
            set_font(run, size=9.5, bold=True, color=INK)

    for values in rows:
        row = table.add_row()
        set_row_keep(row)
        for i, value in enumerate(values):
            cell = row.cells[i]
            cell.text = str(value)
            p = cell.paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if i in center_columns else WD_ALIGN_PARAGRAPH.LEFT
            p.paragraph_format.space_after = Pt(0)
            p.paragraph_format.line_spacing = 1.1
            for run in p.runs:
                set_font(run, size=9.5)
    set_table_geometry(table, widths)
    after = doc.add_paragraph()
    after.paragraph_format.space_after = Pt(2)
    return table


doc = Document()
section = doc.sections[0]
section.page_width = Inches(8.5)
section.page_height = Inches(11)
section.top_margin = Inches(1.0)
section.bottom_margin = Inches(1.0)
section.left_margin = Inches(1.0)
section.right_margin = Inches(1.0)
section.header_distance = Inches(0.492)
section.footer_distance = Inches(0.492)

styles = doc.styles
normal = styles["Normal"]
normal.font.name = "Calibri"
normal._element.rPr.rFonts.set(qn("w:ascii"), "Calibri")
normal._element.rPr.rFonts.set(qn("w:hAnsi"), "Calibri")
normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
normal.font.size = Pt(11)
normal.paragraph_format.space_before = Pt(0)
normal.paragraph_format.space_after = Pt(6)
normal.paragraph_format.line_spacing = 1.25

for name, size, color, before, after in (
    ("Heading 1", 16, BLUE, 18, 10),
    ("Heading 2", 13, BLUE, 14, 7),
    ("Heading 3", 12, DARK_BLUE, 10, 5),
):
    style = styles[name]
    style.font.name = "Calibri"
    style._element.rPr.rFonts.set(qn("w:ascii"), "Calibri")
    style._element.rPr.rFonts.set(qn("w:hAnsi"), "Calibri")
    style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    style.font.size = Pt(size)
    style.font.bold = True
    style.font.color.rgb = RGBColor.from_string(color)
    style.paragraph_format.space_before = Pt(before)
    style.paragraph_format.space_after = Pt(after)
    style.paragraph_format.keep_with_next = True

# Running header/footer.
header = section.header.paragraphs[0]
header.paragraph_format.space_after = Pt(0)
header.paragraph_format.tab_stops.add_tab_stop(Inches(6.5))
rh = header.add_run("机械臂工程接线参考\tSTM32F407VGT6")
set_font(rh, size=9, color=MUTED)
set_paragraph_border_bottom(header, color="D7DBE2", size="4", space="4")

footer = section.footer.paragraphs[0]
footer.alignment = WD_ALIGN_PARAGRAPH.RIGHT
rf = footer.add_run("第 ")
set_font(rf, size=9, color=MUTED)
add_field(footer, "PAGE")
rf = footer.add_run(" 页 / 共 ")
set_font(rf, size=9, color=MUTED)
add_field(footer, "NUMPAGES")
rf = footer.add_run(" 页")
set_font(rf, size=9, color=MUTED)

# Memo masthead.
p = doc.add_paragraph()
p.paragraph_format.space_before = Pt(10)
p.paragraph_format.space_after = Pt(3)
r = p.add_run("硬件接线参考手册")
set_font(r, size=10, bold=True, color=BLUE)

p = doc.add_paragraph()
p.paragraph_format.space_after = Pt(4)
r = p.add_run("STM32F407VGT6机械臂工程接线表")
set_font(r, size=23, bold=True, color=INK)

p = doc.add_paragraph()
p.paragraph_format.space_after = Pt(12)
r = p.add_run("5路舵机 · K230视觉 · 串口调试 · 按键 · 电磁铁 · 蜂鸣器 · 急停")
set_font(r, size=12.5, color=MUTED)

for label, value in (
    ("工程路径", r"D:\STM32\HAL\26"),
    ("适用控制器", "STM32F407VGT6（当前代码引脚定义）"),
    ("串口参数", "USART1/USART2：115200，8N1，3.3V TTL"),
    ("更新日期", "2026-08-01"),
):
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(2)
    r = p.add_run(label + "：")
    set_font(r, size=10.5, bold=True, color=INK)
    r = p.add_run(value)
    set_font(r, size=10.5)

rule = doc.add_paragraph()
rule.paragraph_format.space_after = Pt(8)
set_paragraph_border_bottom(rule, color=BLUE, size="14", space="3")

add_note_box(
    doc,
    "接线总原则",
    "舵机必须使用独立大电流电源。STM32、K230、USB-TTL、舵机电源和继电器控制端必须共地；所有信号按3.3V TTL逻辑连接。",
    warning=True,
)

add_heading(doc, "一、总引脚表", 1)
add_table(
    doc,
    ["模块", "功能", "外设 / 引脚", "有效方式与说明"],
    [
        ("舵机1", "电磁铁末端旋转J1", "TIM1_CH2 / PE11", "50Hz PWM，500~2500us对应0~180°"),
        ("舵机2", "腕部俯仰J2", "TIM1_CH3 / PE13", "50Hz PWM，500~2500us对应0~180°"),
        ("舵机3", "肘部J3", "TIM1_CH4 / PE14", "50Hz PWM，500~2500us对应0~180°"),
        ("舵机4", "肩部J4", "TIM8_CH1 / PC6", "50Hz PWM，500~2500us对应0~180°"),
        ("舵机5", "底座旋转J5", "TIM8_CH2 / PC7", "50Hz PWM，500~2500us对应0~180°"),
        ("K230视觉", "STM32发送/接收视觉数据", "USART1 / PB6、PB7", "115200，8N1，DMA+空闲中断"),
        ("电脑调试", "USB-TTL文本命令", "USART2 / PA2、PA3", "115200，8N1，DMA+空闲中断"),
        ("电磁铁", "继电器控制", "GPIO / PD12", "高电平闭合继电器"),
        ("完成蜂鸣器", "拼图完成提示", "GPIO / PE9", "高电平触发，响300ms后拉低"),
        ("急停预留", "常闭急停输入", "GPIO / PD13", "低电平有效；当前软件未启用"),
        ("10个按键", "任务、测试点和停止", "PE0~PE8、PE10", "内部上拉，按下接GND"),
    ],
    [1400, 2600, 2100, 3260],
    center_columns=(0, 2),
)

add_heading(doc, "二、5路舵机接线", 1)
add_table(
    doc,
    ["编号", "机械作用", "PWM信号", "电源", "地线"],
    [
        ("J1", "电磁铁末端旋转", "PE11", "外部舵机电源正极", "外部电源GND + STM32 GND"),
        ("J2", "腕部俯仰", "PE13", "外部舵机电源正极", "外部电源GND + STM32 GND"),
        ("J3", "肘部", "PE14", "外部舵机电源正极", "外部电源GND + STM32 GND"),
        ("J4", "肩部", "PC6", "外部舵机电源正极", "外部电源GND + STM32 GND"),
        ("J5", "底座旋转", "PC7", "外部舵机电源正极", "外部电源GND + STM32 GND"),
    ],
    [900, 2100, 1200, 2400, 2760],
    center_columns=(0, 2),
)
add_note_box(doc, "舵机供电警告", "不要由STM32开发板的3.3V或小电流5V引脚直接给舵机供电。舵机堵转电流较大，应使用独立电源，并在电源入口配置足够容量的电解电容。", warning=True)

add_heading(doc, "三、K230视觉串口（USART1）", 1)
add_table(
    doc,
    ["STM32端", "K230端", "说明"],
    [
        ("PB6 / USART1_TX", "K230 RX", "STM32向视觉发送3A、3B、3C、AC等单字节命令"),
        ("PB7 / USART1_RX", "K230 TX", "STM32接收52字节视觉坐标帧"),
        ("STM32 GND", "K230 GND", "必须共地"),
    ],
    [2500, 1900, 4960],
    center_columns=(0, 1),
)
add_note_box(doc, "串口交叉连接", "TX必须连接对方RX，RX必须连接对方TX。参数为115200、8位数据、无校验、1位停止位、无硬件流控。", warning=False)

add_heading(doc, "四、电脑USB-TTL调试串口（USART2）", 1)
add_table(
    doc,
    ["STM32端", "USB-TTL端", "说明"],
    [
        ("PA2 / USART2_TX", "USB-TTL RX", "STM32输出状态和文本应答"),
        ("PA3 / USART2_RX", "USB-TTL TX", "电脑发送GOTO、STATUS、TASK等ASCII命令"),
        ("STM32 GND", "USB-TTL GND", "必须共地"),
    ],
    [2500, 1900, 4960],
    center_columns=(0, 1),
)

add_heading(doc, "五、10个按键", 1)
p = doc.add_paragraph("所有按键均使用内部上拉、低电平触发：按键一端接对应GPIO，另一端统一接GND，不需要连接3.3V。")
p.paragraph_format.keep_with_next = True
add_table(
    doc,
    ["按键", "GPIO", "当前功能"],
    [
        ("KEY1", "PE0", "启动任务1，向视觉发送0x3A"),
        ("KEY2", "PE1", "启动任务2，向视觉发送0x3B"),
        ("KEY3", "PE2", "启动任务3，发送0x3C，并使用任务3独立速度参数"),
        ("KEY4", "PE3", "GOTO 0 0 5 0 2000"),
        ("KEY5", "PE4", "GOTO 210 0 5 0 3000"),
        ("KEY6", "PE5", "GOTO 105 -70 1 0 3000"),
        ("KEY7", "PE6", "机械臂用3000ms回到HOME"),
        ("KEY8", "PE7", "软件停止拼图、校准和运动，并关闭电磁铁"),
        ("KEY9", "PE8", "向视觉发送0x3F，启动视觉校准"),
        ("KEY10", "PE10", "向视觉发送0xAD，启动视觉辅助校准"),
    ],
    [1400, 1400, 6560],
    center_columns=(0, 1),
)

add_heading(doc, "六、电磁铁继电器", 1)
add_table(
    doc,
    ["STM32端", "继电器模块端", "功能"],
    [
        ("PD12", "IN", "高电平使继电器闭合"),
        ("STM32 GND", "GND", "控制端共地"),
        ("模块要求的电源", "VCC", "按继电器模块额定电压供电"),
    ],
    [2400, 2400, 4560],
    center_columns=(0, 1),
)
p = doc.add_paragraph("负载端推荐：电磁铁电源正极 → COM；NO → 电磁铁正极；电磁铁负极 → 电磁铁电源负极。")
p.paragraph_format.space_after = Pt(6)
add_note_box(doc, "继电器驱动警告", "若使用裸继电器，必须增加三极管或MOSFET驱动及续流二极管，禁止使用PD12直接驱动继电器线圈。", warning=True)

add_heading(doc, "七、拼图完成蜂鸣器", 1)
add_table(
    doc,
    ["STM32端", "蜂鸣器模块端", "说明"],
    [
        ("PE9", "IN", "高电平触发有源蜂鸣器"),
        ("STM32 GND", "GND", "必须共地"),
        ("模块要求的电源", "VCC", "按蜂鸣器模块额定电压供电"),
    ],
    [2400, 2400, 4560],
    center_columns=(0, 1),
)
p = doc.add_paragraph("当前流程：最后一块放下并关闭电磁铁 → 等待300ms → PE9拉高300ms → PE9自动恢复低电平。")
add_note_box(doc, "蜂鸣器类型", "应使用高电平触发的有源蜂鸣器模块或MOSFET驱动板，不能由PE9直接驱动大电流蜂鸣器或线圈。", warning=True)

add_heading(doc, "八、急停输入（预留）", 1)
p = doc.add_paragraph("预留接法：PD13 → 常闭急停开关 → GND。输入采用内部上拉、低电平有效。")
p.paragraph_format.space_after = Pt(6)
add_note_box(doc, "当前状态", "ARM_ESTOP_ENABLED 当前为0，PD13急停功能尚未启用。需要使用时，先把该宏改为1，确认常闭开关逻辑，再重新编译烧录。", warning=True)

add_heading(doc, "九、SWD下载调试", 1)
add_table(
    doc,
    ["调试器端", "STM32端", "说明"],
    [
        ("SWDIO", "PA13", "数据线"),
        ("SWCLK", "PA14", "时钟线"),
        ("GND", "GND", "共地"),
        ("3.3V参考", "3.3V", "电平参考，不建议由调试器给整机大负载供电"),
        ("NRST（可选）", "NRST", "复位线"),
    ],
    [2200, 1800, 5360],
    center_columns=(0, 1),
)

add_heading(doc, "十、上电前检查清单", 1)
add_table(
    doc,
    ["序号", "检查要求"],
    [
        ("1", "确认PE11、PE13、PE14、PC6、PC7分别连接J1~J5，避免HOME动作撞机。"),
        ("2", "确认舵机由独立大电流电源供电，且与STM32共地。"),
        ("3", "确认K230和USB-TTL均为3.3V TTL，TX/RX交叉连接。"),
        ("4", "确认PD12只接继电器模块输入，不直接驱动线圈。"),
        ("5", "确认PE9只接高电平触发蜂鸣器模块输入。"),
        ("6", "首次上电时保持机械臂工作空间无障碍，并准备使用KEY8停止。"),
    ],
    [900, 8460],
    center_columns=(0,),
)

# Metadata and compatibility settings.
doc.core_properties.title = "STM32F407VGT6机械臂工程接线表"
doc.core_properties.subject = "机械臂控制工程硬件接线参考"
doc.core_properties.author = "Codex"
doc.core_properties.keywords = "STM32F407VGT6, 机械臂, K230, 舵机, 接线表"

settings = doc.settings._element
update_fields = settings.find(qn("w:updateFields"))
if update_fields is None:
    update_fields = OxmlElement("w:updateFields")
    settings.append(update_fields)
update_fields.set(qn("w:val"), "true")

OUT.parent.mkdir(parents=True, exist_ok=True)
doc.save(OUT)
print(OUT)
