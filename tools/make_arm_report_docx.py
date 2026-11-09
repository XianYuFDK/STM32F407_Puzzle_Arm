import os
import zipfile
from datetime import datetime
from xml.sax.saxutils import escape


OUT_DOCX = r"D:\STM32\HAL\26\docs\机械臂当前代码算法与控制流程分析.docx"

NS_W = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
NS_R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"


def x(text):
    return escape(str(text), {"\"": "&quot;"})


def r(text, bold=False, color=None, size=None):
    props = []
    if bold:
        props.append("<w:b/>")
    if color:
        props.append(f'<w:color w:val="{color}"/>')
    if size:
        half = int(size * 2)
        props.append(f'<w:sz w:val="{half}"/><w:szCs w:val="{half}"/>')
    prop_xml = f"<w:rPr>{''.join(props)}</w:rPr>" if props else ""
    return f"<w:r>{prop_xml}<w:t>{x(text)}</w:t></w:r>"


def p(text="", style=None, before=None, after=None, line=None, keep=False):
    ppr = []
    if style:
        ppr.append(f'<w:pStyle w:val="{style}"/>')
    spacing = []
    if before is not None:
        spacing.append(f'w:before="{before}"')
    if after is not None:
        spacing.append(f'w:after="{after}"')
    if line is not None:
        spacing.append(f'w:line="{line}" w:lineRule="auto"')
    if spacing:
        ppr.append(f"<w:spacing {' '.join(spacing)}/>")
    if keep:
        ppr.append("<w:keepNext/>")
    ppr_xml = f"<w:pPr>{''.join(ppr)}</w:pPr>" if ppr else ""
    return f"<w:p>{ppr_xml}{r(text)}</w:p>"


def bullet(text):
    return (
        '<w:p><w:pPr><w:pStyle w:val="ListParagraph"/>'
        '<w:numPr><w:ilvl w:val="0"/><w:numId w:val="1"/></w:numPr>'
        '<w:spacing w:after="80" w:line="280" w:lineRule="auto"/>'
        f"</w:pPr>{r(text)}</w:p>"
    )


def num(text, num_id=2):
    return (
        '<w:p><w:pPr><w:pStyle w:val="ListParagraph"/>'
        f'<w:numPr><w:ilvl w:val="0"/><w:numId w:val="{num_id}"/></w:numPr>'
        '<w:spacing w:after="80" w:line="280" w:lineRule="auto"/>'
        f"</w:pPr>{r(text)}</w:p>"
    )


def code(text):
    return (
        '<w:p><w:pPr><w:pStyle w:val="CodeBlock"/>'
        '<w:spacing w:before="80" w:after="120"/></w:pPr>'
        f"{r(text)}</w:p>"
    )


def table(rows, widths):
    width_sum = sum(widths)
    grid = "".join(f'<w:gridCol w:w="{w}"/>' for w in widths)
    out = [
        "<w:tbl>",
        '<w:tblPr><w:tblStyle w:val="TableGrid"/>'
        f'<w:tblW w:w="{width_sum}" w:type="dxa"/>'
        '<w:tblInd w:w="120" w:type="dxa"/>'
        '<w:tblBorders>'
        '<w:top w:val="single" w:sz="4" w:space="0" w:color="D0D7DE"/>'
        '<w:left w:val="single" w:sz="4" w:space="0" w:color="D0D7DE"/>'
        '<w:bottom w:val="single" w:sz="4" w:space="0" w:color="D0D7DE"/>'
        '<w:right w:val="single" w:sz="4" w:space="0" w:color="D0D7DE"/>'
        '<w:insideH w:val="single" w:sz="4" w:space="0" w:color="D0D7DE"/>'
        '<w:insideV w:val="single" w:sz="4" w:space="0" w:color="D0D7DE"/>'
        '</w:tblBorders>'
        '<w:tblCellMar><w:top w:w="80" w:type="dxa"/>'
        '<w:left w:w="120" w:type="dxa"/><w:bottom w:w="80" w:type="dxa"/>'
        '<w:right w:w="120" w:type="dxa"/></w:tblCellMar>'
        '<w:tblLayout w:type="fixed"/></w:tblPr>',
        f"<w:tblGrid>{grid}</w:tblGrid>",
    ]
    for row_index, row in enumerate(rows):
        out.append("<w:tr>")
        for cell, wdt in zip(row, widths):
            fill = '<w:shd w:fill="F2F4F7"/>' if row_index == 0 else ""
            out.append(
                "<w:tc>"
                f'<w:tcPr><w:tcW w:w="{wdt}" w:type="dxa"/>{fill}</w:tcPr>'
                f'{p(cell, style="TableText", after=0)}'
                "</w:tc>"
            )
        out.append("</w:tr>")
    out.append("</w:tbl>")
    return "".join(out)


def page_break():
    return '<w:p><w:r><w:br w:type="page"/></w:r></w:p>'


def document_xml():
    generated = datetime.now().strftime("%Y-%m-%d")
    body = []

    body.append(p("机械臂当前代码算法与控制流程分析", style="Title", after=120))
    body.append(p("用于电赛拼图装置 STM32F407 机械臂控制程序报告撰写", style="Subtitle", after=220))
    body.append(table([
        ("项目", "内容"),
        ("工程路径", r"D:\STM32\HAL\26"),
        ("控制器", "STM32F407VET6/VGT6，HAL + FreeRTOS"),
        ("机械结构", "幻尔开源六路舵机机械臂改五路，末端改电磁铁"),
        ("视觉模块", "K230，USART1发送任务触发并返回拼图坐标帧"),
        ("生成日期", generated),
    ], [1900, 7460]))

    body.append(p("一、系统总体方案", style="Heading1"))
    body.append(p(
        "当前程序采用“视觉一次识别、单片机完成坐标补偿和逆运动学、机械臂开环执行”的总体方案。"
        "K230视觉端负责识别铁片的抓取点、目标放置点和旋转角；STM32接收视觉结果后，完成协议解析、"
        "坐标修正、逆运动学求解、舵机平滑插补、电磁铁控制以及安全保护。"
    ))
    body.append(table([
        ("模块", "主要作用", "对应代码"),
        ("视觉通信", "接收USART1视觉帧，校验并回复ACK", "arm_comm.c, arm_puzzle_protocol.c"),
        ("调试命令", "USART2文本命令，支持GOTO、IK、VREAD、VFRAME等", "arm_debug_cli.c"),
        ("坐标与逆解", "坐标补偿、底座偏航计算、二连杆逆运动学", "arm_kinematics.c"),
        ("抓放流程", "根据视觉计划生成抓取、抬升、转运、放置动作", "arm_puzzle.c"),
        ("运动插补", "20ms周期平滑更新五路舵机角度", "arm_motion.c"),
        ("底层驱动", "PWM输出、串口DMA、电磁铁GPIO", "arm_servo.c, BSP模块"),
        ("安全保护", "急停、电磁铁超时、通信看门狗", "arm_safety.c"),
    ], [1500, 3900, 3960]))

    body.append(p("二、视觉通信与协议解析", style="Heading1"))
    body.append(p(
        "视觉端和STM32之间通过USART1通信。STM32发送单字节命令启动视觉识别：任务1发送0x3A，任务2发送0x3B。"
        "K230返回固定52字节结果帧，帧内包含每个铁片的编号、抓取坐标、放置坐标和旋转角。"
    ))
    body.append(table([
        ("字段", "含义"),
        ("帧头", "AA 55"),
        ("版本号", "协议版本，当前为01"),
        ("序号", "视觉帧序列号"),
        ("数量", "本次返回的铁片数量"),
        ("记录区", "每片包含ID、pick_x、pick_y、place_x、place_y、rotate"),
        ("校验和", "对固定区间字节累加取低8位"),
        ("帧尾", "55 CC"),
    ], [1800, 7560]))
    body.append(p(
        "解析器按字节流搜索AA 55帧头，累计满52字节后检查版本、数量范围、坐标范围、旋转角范围、校验和和帧尾。"
        "合法帧会生成ArmPuzzlePlan；非法帧只累计错误计数，不启动机械臂。"
    ))
    body.append(p(
        "通信层还使用FNV-1a哈希作为计划签名，用于识别ACK丢失导致的重复视觉包：同一任务的重复包只回复0xAC，"
        "不重复执行机械臂动作。"
    ))

    body.append(p("三、坐标系与几何模型", style="Heading1"))
    body.append(p(
        "题面坐标原点定义为A4左边线与橙色中线交点，X轴向A4右侧为正，Y轴向A4下方为正，"
        "Z表示电磁铁吸附面相对A4纸面的高度。机械臂底座位于题面原点后方，程序中按底座中心投影"
        "(-70, 0) mm建模。"
    ))
    body.append(table([
        ("参数", "当前值", "说明"),
        ("肩轴到肘轴", "165 mm", "仿真原长度基础上加长约60 mm"),
        ("肘轴到腕轴", "153 mm", "仿真原长度基础上加长约63 mm"),
        ("腕轴到电磁铁中心", "80 mm", "末端工具长度"),
        ("肩轴高度", "90 mm", "相对底座安装平面"),
        ("底座中心坐标", "(-70, 0) mm", "A4坐标系下的底座投影"),
        ("A4平面高度补偿", "5 mm", "程序当前实际取值"),
        ("工具目标俯仰", "-90 deg", "使电磁铁吸附面与A4平面平行"),
    ], [2300, 2100, 4960]))

    body.append(p("四、坐标补偿算法", style="Heading1"))
    body.append(p(
        "由于舵机虚位、重力下坠、连杆弹性、安装误差和底座摆放误差会造成实际落点与理论坐标不一致，"
        "程序在逆运动学前加入多层实测补偿。该补偿不是针对某个点写死舵机角，而是将实测误差转换为连续曲线，"
        "使GOTO调试、视觉抓放和校准点共用同一套坐标修正模型。"
    ))
    body.append(code(
        "视觉/调试坐标 -> XY二维残差补偿 -> X轴反插值 -> Y轴反插值 -> "
        "A4九宫格双线性补偿 -> 右半区残差补偿 -> P3/中段X补偿 -> 逆运动学"
    ))
    body.append(table([
        ("补偿层", "算法", "目的"),
        ("X轴反标定", "分段线性反插值", "根据实测X落点反推应输入的命令X"),
        ("Y轴反标定", "对|Y|分段线性反插值并恢复符号", "修正上下半区对称偏差"),
        ("XY残差补偿", "两行三列双线性插值", "修正X/Y耦合造成的二维误差"),
        ("A4九宫格补偿", "3x3双线性插值，当前比例0.6", "按九个标定点修正全局落点"),
        ("右半区残差", "X>105mm后线性介入", "修正右半区约1cm的X/Y偏差"),
        ("远点俯仰补偿", "按水平距离线性增加工具俯仰", "抵消远点重力下坠和舵机虚位"),
    ], [2100, 3000, 4260]))

    body.append(p("五、逆运动学求解", style="Heading1"))
    body.append(p(
        "逆解函数ArmKinematics_SolveBoardPose接收A4坐标、目标高度和末端电磁铁旋转角，输出五个舵机的目标角度。"
        "该机械臂可分解为底座水平偏航和侧视平面二连杆结构。"
    ))
    for item in [
        "将A4坐标经过补偿后转换到底座坐标，计算dx、dy。",
        "由atan2(dy, dx)求底座偏航角J5。",
        "由水平距离得到侧视平面x，由目标Z和肩轴高度得到侧视平面z。",
        "固定工具俯仰角为-90°，并扣除腕轴到电磁铁中心的80mm工具长度，得到腕轴目标点。",
        "使用余弦定理计算肘角，再计算肩角，最后由工具姿态约束求腕角。",
        "将数学关节角映射成实际舵机角，并检查0~180°软件限位。",
    ]:
        body.append(num(item, 2))
    body.append(code("cos(elbow) = (x^2 + z^2 - L1^2 - L2^2) / (2 * L1 * L2)"))
    body.append(p(
        "程序会尝试两个肘部分支，优先选择与实机弯曲方向一致的负肘分支。若任一关节超出软件限位，"
        "则认为该坐标不可达，返回IK_UNREACHABLE。"
    ))

    body.append(p("六、舵机角度映射与末端平行控制", style="Heading1"))
    body.append(p(
        "逆解得到的是数学关节角，必须映射成实际五路舵机的0~180°角度。当前编号为：J1末端电磁铁旋转，"
        "J2腕部俯仰，J3肘部，J4肩部，J5底座旋转。"
    ))
    body.append(table([
        ("关节", "机械作用", "当前HOME", "备注"),
        ("J1", "电磁铁末端旋转", "90°", "用于贴片方向调整"),
        ("J2", "腕部/末端俯仰", "173°", "与J3/J4共同保证末端平行"),
        ("J3", "肘部", "90°", "软件最大限位175°"),
        ("J4", "肩部", "90°", "J4=87°约为实机垂直"),
        ("J5", "底座旋转", "0°", "90°约正对A4，0°朝原点右侧"),
    ], [1200, 2600, 1700, 3860]))
    body.append(p(
        "为保证电磁铁吸附面与A4纸面平行，程序在J3、J4实机标定后，将标定引入的姿态差动态补偿到J2。"
        "因此不是简单固定某个腕部角度，而是让J2、J3、J4组合满足末端俯仰约束。"
    ))

    body.append(p("七、抓取与放置控制流程", style="Heading1"))
    body.append(p(
        "视觉计划进入ArmPuzzle任务后，程序会先对全部铁片的抓取点、放置点、安全高度和表面高度进行预逆解。"
        "只要有任意一步不可达，整套任务直接拒绝执行，避免执行到中途停在纸面附近。"
    ))
    for item in [
        "移动到抓取点上方安全高度。",
        "竖直下降到抓取高度。",
        "等待机械臂稳定，避免抖动时提前吸偏。",
        "PD12输出高电平，继电器闭合，电磁铁吸附。",
        "吸合保持一段时间，确保铁片被吸稳。",
        "保持抓取瞬间腕部角度不变，直接抬升到安全高度。",
        "移动到放置点上方安全高度。",
        "下降到放置高度。",
        "等待机械臂和铁片稳定。",
        "关闭电磁铁释放铁片，随后抬升。",
        "所有铁片完成后返回HOME。",
    ]:
        body.append(num(item, 3))
    body.append(table([
        ("动作参数", "当前值"),
        ("运动安全高度", "60 mm"),
        ("抓取高度", "-1 mm"),
        ("放置高度", "1 mm"),
        ("抓取前稳定延时", "1500 ms"),
        ("吸合后等待", "900 ms"),
        ("放置前稳定延时", "1500 ms"),
        ("释放后等待", "500 ms"),
        ("单次计划超时", "115000 ms"),
    ], [3000, 6360]))

    body.append(p("八、铁片旋转角处理", style="Heading1"))
    body.append(p(
        "视觉给出的rotate表示铁片从抓取姿态到目标姿态需要的世界坐标旋转角。由于机械臂从抓取点转到放置点时，"
        "底座自身偏航也会带动铁片旋转，因此程序先扣除底座转角差，再把剩余角度平均分配给抓取前和放置前的J1末端旋转。"
    ))
    body.append(code("required_rotation = vision_rotate - (place_base_yaw - pick_base_yaw)"))
    body.append(code("pick_magnet_relative = -0.5 * required_rotation"))
    body.append(code("place_magnet_relative = +0.5 * required_rotation"))
    body.append(p(
        "这样可以尽量把J1控制在0~180°范围内，降低单侧转角过大导致超限的概率。"
    ))

    body.append(p("九、运动插补算法", style="Heading1"))
    body.append(p(
        "所有舵机运动都由ArmMotion任务统一执行。任务从当前角度读起点，从命令读取终点，按20ms周期逐步写入PWM目标角度。"
        "普通动作使用三次平滑步进，起点和终点速度均为0，可以减少机械臂启动、停止和大角度运动时的冲击。"
    ))
    body.append(code("smooth = 3 * t^2 - 2 * t^3"))
    body.append(p(
        "之前出现“给了3000ms但突然移动”的问题，代码中已改为每步相对延时，避免任务被打断后使用绝对周期追赶，"
        "从而把多个PWM目标压缩到很短时间内写入。"
    ))

    body.append(p("十、FreeRTOS任务划分", style="Heading1"))
    body.append(table([
        ("任务", "优先级", "职责"),
        ("ArmSafety", "48", "急停、电磁铁最长吸合时间、通信看门狗"),
        ("ArmMotion", "40", "舵机角度插补和PWM目标更新"),
        ("ArmComm", "32", "USART1视觉帧、USART2调试命令、事件回传"),
        ("ArmPuzzle", "30", "拼图计划预解算与抓放流程执行"),
        ("ArmCalib", "30", "A4六点校准与点位检查"),
    ], [1900, 1300, 6160]))
    body.append(p(
        "系统启动时先初始化电磁铁、PWM、串口DMA等硬件，然后创建通信、运动、校准、拼图和安全任务。"
        "通信任务只负责解析和提交计划，真正的舵机运动由运动任务统一执行，因此模块之间耦合较低。"
    ))

    body.append(p("十一、调试与校准机制", style="Heading1"))
    body.append(p(
        "USART2作为电脑调试口，采用纯文本命令，便于使用VOFA+或串口助手直接查看。"
        "GOTO命令用于验证A4坐标到机械臂落点是否准确；IK命令只输出理论舵机角，不驱动机械臂；"
        "VREAD用于读取真实视觉帧并解析坐标但不启动机械臂。"
    ))
    body.append(table([
        ("命令", "作用"),
        ("IK X Y Z M", "只计算指定A4坐标对应的五路舵机角"),
        ("GOTO X Y Z M T", "移动到指定A4坐标，用于落点调试"),
        ("TASK1 / TASK2", "触发视觉任务并执行抓放"),
        ("VREAD 1 / VREAD 2", "触发视觉并仅解析坐标，不运动"),
        ("VFRAME 1 HEX", "从调试口注入模拟视觉帧并执行"),
        ("CALIB CHECK", "计算A4六点是否可达并输出舵机角"),
        ("CALIB START", "执行原点、HOME、P1~P6逐点检查"),
        ("STATUS", "查看当前舵机、视觉帧、错误和任务状态"),
        ("STOP", "停止运动并关闭电磁铁"),
    ], [2500, 6860]))
    body.append(p(
        "当前CALIB START流程不是连续沿边框巡线，而是先到(0,0)，回HOME，再依次到P1~P6，每个点之间都回HOME。"
        "这样方便观察坐标误差，也减少连续运动回差对下一个测点的影响。"
    ))

    body.append(p("十二、安全保护设计", style="Heading1"))
    for item in [
        "舵机角度在输出前统一检查，任何关节超出限位则整组角度不输出。",
        "拼图任务执行前预解算全部动作点，任一点不可达则拒绝整套任务。",
        "STOP命令会停止拼图和校准任务，并关闭电磁铁。",
        "电磁铁最长吸合时间默认为10000ms，超过后触发保护。",
        "USART1正式视觉口只回复0xAC，不混发调试文本，避免干扰K230协议。",
        "未经TASK1、TASK2或VREAD触发的残留视觉帧不会启动机械臂。",
    ]:
        body.append(bullet(item))

    body.append(p("十三、报告可用总结", style="Heading1"))
    body.append(p(
        "本系统采用视觉识别与嵌入式运动控制分层设计。视觉模块输出铁片抓取坐标、放置坐标和旋转角；"
        "STM32对坐标进行实测标定补偿后，基于五自由度舵机机械臂模型进行逆运动学求解，得到各舵机目标角度。"
        "运动控制采用FreeRTOS任务化架构，利用周期插补和平滑步进实现舵机缓慢稳定运动，并通过继电器控制电磁铁完成铁片吸附与释放。"
        "系统同时设计了串口调试、视觉帧模拟、A4点位校准、重复帧过滤和超时保护机制，提高了调试效率与比赛运行可靠性。"
    ))

    body.append(p("十四、后续调参建议", style="Heading1"))
    for item in [
        "若视觉坐标和GOTO坐标不一致，优先使用VREAD确认视觉原始坐标，再用GOTO验证同一坐标的机械落点。",
        "若近点吸不上、远点蹭纸，优先调整抓取/放置高度和A4平面高度补偿，再考虑开启表面高度表。",
        "若某一区域整体偏移，优先修改坐标补偿表；若单个关节姿态不对，再修改J2/J3/J4映射零位和比例。",
        "若远点电磁铁不平行，优先小幅调整远点俯仰补偿最大角，避免把坐标补偿和姿态补偿混在一起调。",
        "每次修改参数后建议按CALIB CHECK、GOTO中心点、GOTO四角点、VREAD视觉读数的顺序验证。",
    ]:
        body.append(bullet(item))

    sect = (
        '<w:sectPr><w:pgSz w:w="12240" w:h="15840"/>'
        '<w:pgMar w:top="1440" w:right="1440" w:bottom="1440" w:left="1440" '
        'w:header="708" w:footer="708" w:gutter="0"/>'
        '</w:sectPr>'
    )
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        f'<w:document xmlns:w="{NS_W}" xmlns:r="{NS_R}"><w:body>'
        + "".join(body)
        + sect
        + "</w:body></w:document>"
    )


def styles_xml():
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="{NS_W}">
  <w:docDefaults>
    <w:rPrDefault><w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:sz w:val="22"/><w:szCs w:val="22"/><w:color w:val="000000"/></w:rPr></w:rPrDefault>
    <w:pPrDefault><w:pPr><w:spacing w:after="120" w:line="264" w:lineRule="auto"/></w:pPr></w:pPrDefault>
  </w:docDefaults>
  <w:style w:type="paragraph" w:default="1" w:styleId="Normal">
    <w:name w:val="Normal"/><w:qFormat/>
    <w:pPr><w:spacing w:after="120" w:line="264" w:lineRule="auto"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:sz w:val="22"/><w:szCs w:val="22"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Title">
    <w:name w:val="Title"/><w:basedOn w:val="Normal"/><w:qFormat/>
    <w:pPr><w:spacing w:before="0" w:after="120"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:b/><w:color w:val="0B2545"/><w:sz w:val="48"/><w:szCs w:val="48"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Subtitle">
    <w:name w:val="Subtitle"/><w:basedOn w:val="Normal"/><w:qFormat/>
    <w:pPr><w:spacing w:after="240"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:color w:val="555555"/><w:sz w:val="24"/><w:szCs w:val="24"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Heading1">
    <w:name w:val="heading 1"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/>
    <w:pPr><w:keepNext/><w:spacing w:before="320" w:after="160" w:line="264" w:lineRule="auto"/><w:outlineLvl w:val="0"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:b/><w:color w:val="2E74B5"/><w:sz w:val="32"/><w:szCs w:val="32"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Heading2">
    <w:name w:val="heading 2"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:qFormat/>
    <w:pPr><w:keepNext/><w:spacing w:before="240" w:after="120" w:line="264" w:lineRule="auto"/><w:outlineLvl w:val="1"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:b/><w:color w:val="2E74B5"/><w:sz w:val="26"/><w:szCs w:val="26"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="ListParagraph">
    <w:name w:val="List Paragraph"/><w:basedOn w:val="Normal"/><w:qFormat/>
    <w:pPr><w:ind w:left="720" w:hanging="360"/><w:spacing w:after="80" w:line="280" w:lineRule="auto"/></w:pPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="CodeBlock">
    <w:name w:val="Code Block"/><w:basedOn w:val="Normal"/>
    <w:pPr><w:spacing w:before="80" w:after="120"/><w:shd w:fill="F4F6F9"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Consolas" w:hAnsi="Consolas" w:eastAsia="Microsoft YaHei"/><w:sz w:val="20"/><w:szCs w:val="20"/><w:color w:val="1F3A5F"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="TableText">
    <w:name w:val="Table Text"/><w:basedOn w:val="Normal"/>
    <w:pPr><w:spacing w:after="0" w:line="260" w:lineRule="auto"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:sz w:val="20"/><w:szCs w:val="20"/></w:rPr>
  </w:style>
</w:styles>'''


def numbering_xml():
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:numbering xmlns:w="{NS_W}">
  <w:abstractNum w:abstractNumId="1">
    <w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="bullet"/><w:lvlText w:val="•"/>
      <w:pPr><w:tabs><w:tab w:val="num" w:pos="720"/></w:tabs><w:ind w:left="720" w:hanging="360"/></w:pPr>
    </w:lvl>
  </w:abstractNum>
  <w:num w:numId="1"><w:abstractNumId w:val="1"/></w:num>
  <w:abstractNum w:abstractNumId="2">
    <w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="decimal"/><w:lvlText w:val="%1."/>
      <w:pPr><w:tabs><w:tab w:val="num" w:pos="720"/></w:tabs><w:ind w:left="720" w:hanging="360"/></w:pPr>
    </w:lvl>
  </w:abstractNum>
  <w:num w:numId="2"><w:abstractNumId w:val="2"/></w:num>
  <w:num w:numId="3"><w:abstractNumId w:val="2"/></w:num>
</w:numbering>'''


def content_types():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
  <Override PartName="/word/numbering.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.numbering+xml"/>
  <Override PartName="/word/settings.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.settings+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>'''


def rels():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
</Relationships>'''


def doc_rels():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/numbering" Target="numbering.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/settings" Target="settings.xml"/>
</Relationships>'''


def settings_xml():
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:settings xmlns:w="{NS_W}"><w:defaultTabStop w:val="720"/></w:settings>'''


def core_xml():
    now = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:dcterms="http://purl.org/dc/terms/" xmlns:dcmitype="http://purl.org/dc/dcmitype/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>机械臂当前代码算法与控制流程分析</dc:title>
  <dc:creator>Codex</dc:creator>
  <cp:lastModifiedBy>Codex</cp:lastModifiedBy>
  <dcterms:created xsi:type="dcterms:W3CDTF">{now}</dcterms:created>
  <dcterms:modified xsi:type="dcterms:W3CDTF">{now}</dcterms:modified>
</cp:coreProperties>'''


def app_xml():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">
  <Application>Codex OOXML Generator</Application>
</Properties>'''


def main():
    os.makedirs(os.path.dirname(OUT_DOCX), exist_ok=True)
    with zipfile.ZipFile(OUT_DOCX, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", content_types())
        z.writestr("_rels/.rels", rels())
        z.writestr("word/document.xml", document_xml())
        z.writestr("word/styles.xml", styles_xml())
        z.writestr("word/numbering.xml", numbering_xml())
        z.writestr("word/settings.xml", settings_xml())
        z.writestr("word/_rels/document.xml.rels", doc_rels())
        z.writestr("docProps/core.xml", core_xml())
        z.writestr("docProps/app.xml", app_xml())
    print(OUT_DOCX)


if __name__ == "__main__":
    main()
