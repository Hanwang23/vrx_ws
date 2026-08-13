const path = require('path');
const sharp = require('sharp');
const PptxGenJS = require('pptxgenjs');

const ROOT = '/home/han/Ai_ws/Study/vrx_ws';
const OUT = path.join(ROOT, 'group_meeting_ppt');
const ASSETS = path.join(OUT, 'assets');

const FONT = 'Noto Sans CJK SC';
const C = {
  bg: 'F4F7F9',
  white: 'FFFFFF',
  navy: '17324D',
  dark: '102534',
  text: '25313B',
  muted: '60707D',
  line: 'D5DEE5',
  pale: 'E8EEF3',
  teal: '247F78',
  blue: '2C78A5',
  green: '2E8B62',
  orange: 'D67A2D',
  red: 'B74C46',
  cyan: '1AAFC1',
};

const SIM_SRC = path.join(ROOT, 'src/vrx-humble/images/sydney_regatta_gzsim.png');
const LIDAR_SRC = path.join(ROOT, 'han_usv_controller/lidar_shoreline_view.png');
const PATH_SRC = path.join(ROOT, 'han_usv_controller/path_tracking_view.png');
const SIM_TITLE = path.join(ASSETS, 'sim_title.png');
const SIM_CARD = path.join(ASSETS, 'sim_card.png');
const LIDAR_TITLE = path.join(ASSETS, 'lidar_title.png');
const LIDAR_CARD = path.join(ASSETS, 'lidar_card.png');
const PATH_CARD = path.join(ASSETS, 'path_card.png');

async function prepareAssets() {
  await sharp(SIM_SRC).resize(1600, 900, { fit: 'cover', position: 'center' }).png().toFile(SIM_TITLE);
  await sharp(SIM_SRC).resize(1200, 800, { fit: 'cover', position: 'center' }).png().toFile(SIM_CARD);
  await sharp(LIDAR_SRC)
    .extract({ left: 560, top: 133, width: 1335, height: 910 })
    .resize(1600, 900, { fit: 'cover', position: 'center' })
    .png().toFile(LIDAR_TITLE);
  await sharp(LIDAR_SRC)
    .extract({ left: 560, top: 133, width: 1335, height: 910 })
    .resize(1200, 800, { fit: 'cover', position: 'center' })
    .png().toFile(LIDAR_CARD);
  await sharp(PATH_SRC)
    .extract({ left: 910, top: 181, width: 1010, height: 899 })
    .resize(1200, 800, { fit: 'cover', position: 'center' })
    .png().toFile(PATH_CARD);
}

function newDeck(title, subject) {
  const pptx = new PptxGenJS();
  pptx.layout = 'LAYOUT_WIDE';
  pptx.author = 'HanW';
  pptx.company = 'VRX WAM-V Project';
  pptx.subject = subject;
  pptx.title = title;
  pptx.lang = 'zh-CN';
  pptx.theme = {
    headFontFace: FONT,
    bodyFontFace: FONT,
    lang: 'zh-CN',
  };
  pptx.defineSlideMaster({
    title: 'CONTENT',
    background: { color: C.bg },
    objects: [],
  });
  return pptx;
}

function addTitleSlide(pptx, imagePath, meeting, title, subtitle, tags, storyline, notes) {
  const slide = pptx.addSlide();
  slide.addImage({ path: imagePath, x: 0, y: 0, w: 13.333, h: 7.5 });
  slide.addShape(pptx.ShapeType.rect, {
    x: 0, y: 0, w: 13.333, h: 7.5,
    line: { color: C.dark, transparency: 100 },
    fill: { color: C.dark, transparency: 28 },
  });
  slide.addShape(pptx.ShapeType.rect, {
    x: 0.68, y: 1.18, w: 0.08, h: 3.4,
    line: { color: C.orange, transparency: 100 }, fill: { color: C.orange },
  });
  slide.addText(title, {
    x: 1.02, y: 1.38, w: 10.9, h: 1.35,
    fontFace: FONT, fontSize: 29, bold: true, color: C.white,
    margin: 0, breakLine: false, valign: 'mid', fit: 'shrink',
  });
  slide.addText(subtitle, {
    x: 1.04, y: 3.02, w: 9.8, h: 0.48,
    fontFace: FONT, fontSize: 17, color: 'E9F1F5', margin: 0,
  });
  let tagX = 1.04;
  tags.forEach((tag, idx) => {
    const widths = [1.45, 1.7, 1.7, 1.7];
    const w = widths[idx] || 1.55;
    slide.addShape(pptx.ShapeType.rect, {
      x: tagX, y: 3.78, w, h: 0.42,
      line: { color: C.white, transparency: 72, width: 0.8 },
      fill: { color: C.dark, transparency: 38 },
    });
    slide.addText(tag, {
      x: tagX, y: 3.82, w, h: 0.28,
      fontFace: FONT, fontSize: 10.5, color: C.white, align: 'center', margin: 0,
    });
    tagX += w + 0.18;
  });
  slide.addText(`汇报主线：${storyline}`, {
    x: 1.04, y: 4.48, w: 9.9, h: 0.3,
    fontFace: FONT, fontSize: 11.5, color: 'E9F1F5', margin: 0,
  });
  slide.addText(`汇报人：HanW  |  2026.07.17`, {
    x: 1.04, y: 6.72, w: 5.2, h: 0.28,
    fontFace: FONT, fontSize: 11, color: C.white, margin: 0,
  });
  slide.addText(meeting, {
    x: 9.85, y: 6.66, w: 2.45, h: 0.32,
    fontFace: FONT, fontSize: 11.5, bold: true, color: C.white, align: 'right', margin: 0,
  });
  slide.addNotes(notes);
  return slide;
}

function addContentBase(pptx, title, meeting, page) {
  const slide = pptx.addSlide('CONTENT');
  slide.background = { color: C.bg };
  slide.addShape(pptx.ShapeType.rect, {
    x: 0, y: 0, w: 13.333, h: 0.12,
    line: { color: C.teal, transparency: 100 }, fill: { color: C.teal },
  });
  slide.addText(title, {
    x: 0.62, y: 0.34, w: 10.7, h: 0.48,
    fontFace: FONT, fontSize: 23.5, bold: true, color: C.navy, margin: 0,
  });
  slide.addText(meeting, {
    x: 10.75, y: 0.42, w: 1.95, h: 0.24,
    fontFace: FONT, fontSize: 9.5, color: C.muted, align: 'right', margin: 0,
  });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.62, y: 0.94, w: 12.08, h: 0,
    line: { color: C.line, width: 1 },
  });
  slide.addText(`VRX WAM-V 自主航行系统  |  ${meeting}`, {
    x: 0.62, y: 7.07, w: 5.8, h: 0.2,
    fontFace: FONT, fontSize: 8.5, color: C.muted, margin: 0,
  });
  slide.addText(String(page), {
    x: 12.35, y: 7.06, w: 0.35, h: 0.2,
    fontFace: FONT, fontSize: 9, color: C.muted, align: 'right', margin: 0,
  });
  return slide;
}

function addCard(slide, pptx, x, y, w, h, accent, index, title, body) {
  slide.addShape(pptx.ShapeType.rect, {
    x, y, w, h,
    line: { color: C.line, width: 1 }, fill: { color: C.white },
  });
  slide.addShape(pptx.ShapeType.rect, {
    x, y, w: 0.09, h,
    line: { color: accent, transparency: 100 }, fill: { color: accent },
  });
  slide.addText(index, {
    x: x + 0.25, y: y + 0.2, w: 0.55, h: 0.32,
    fontFace: FONT, fontSize: 18, bold: true, color: accent, margin: 0,
  });
  slide.addText(title, {
    x: x + 0.88, y: y + 0.2, w: w - 1.1, h: 0.3,
    fontFace: FONT, fontSize: 14.5, bold: true, color: C.navy, margin: 0,
  });
  slide.addText(body, {
    x: x + 0.25, y: y + 0.68, w: w - 0.5, h: h - 0.84,
    fontFace: FONT, fontSize: 11.5, color: C.text, margin: 0,
    breakLine: false, valign: 'top', fit: 'shrink',
  });
}

function addFlowArrow(slide, pptx, x, y, w = 0.32) {
  slide.addShape(pptx.ShapeType.chevron, {
    x, y, w, h: 0.45,
    line: { color: C.line, transparency: 100 }, fill: { color: 'A9B8C4' },
  });
}

function addPipelineBox(slide, pptx, x, y, w, title, subtitle, color) {
  slide.addShape(pptx.ShapeType.rect, {
    x, y, w, h: 1.1,
    line: { color, width: 1.2 }, fill: { color: C.white },
  });
  slide.addShape(pptx.ShapeType.rect, {
    x, y, w, h: 0.12,
    line: { color, transparency: 100 }, fill: { color },
  });
  slide.addText(title, {
    x: x + 0.1, y: y + 0.28, w: w - 0.2, h: 0.3,
    fontFace: FONT, fontSize: 12.5, bold: true, color: C.navy, align: 'center', margin: 0,
  });
  slide.addText(subtitle, {
    x: x + 0.1, y: y + 0.66, w: w - 0.2, h: 0.23,
    fontFace: FONT, fontSize: 8.8, color: C.muted, align: 'center', margin: 0,
  });
}

function addStateBox(slide, pptx, x, title, detail, color) {
  slide.addShape(pptx.ShapeType.rect, {
    x, y: 1.47, w: 1.73, h: 1.08,
    line: { color, width: 1.1 }, fill: { color: C.white },
  });
  slide.addText(title, {
    x: x + 0.08, y: 1.71, w: 1.57, h: 0.27,
    fontFace: FONT, fontSize: 12, bold: true, color, align: 'center', margin: 0,
  });
  slide.addText(detail, {
    x: x + 0.08, y: 2.1, w: 1.57, h: 0.22,
    fontFace: FONT, fontSize: 8.7, color: C.muted, align: 'center', margin: 0,
  });
}

function addContributionColumn(slide, pptx, x, color, title, subtitle, items) {
  slide.addShape(pptx.ShapeType.rect, {
    x, y: 1.38, w: 3.77, h: 4.55,
    line: { color: C.line, width: 1 }, fill: { color: C.white },
  });
  slide.addShape(pptx.ShapeType.rect, {
    x, y: 1.38, w: 3.77, h: 0.13,
    line: { color, transparency: 100 }, fill: { color },
  });
  slide.addText(title, {
    x: x + 0.28, y: 1.72, w: 3.2, h: 0.34,
    fontFace: FONT, fontSize: 16, bold: true, color, margin: 0,
  });
  slide.addText(subtitle, {
    x: x + 0.28, y: 2.13, w: 3.2, h: 0.28,
    fontFace: FONT, fontSize: 9.7, color: C.muted, margin: 0,
  });
  slide.addShape(pptx.ShapeType.line, {
    x: x + 0.28, y: 2.57, w: 3.2, h: 0,
    line: { color: C.line, width: 0.8 },
  });
  slide.addText(items.map((item) => `• ${item}`).join('\n'), {
    x: x + 0.3, y: 2.82, w: 3.12, h: 2.78,
    fontFace: FONT, fontSize: 11.2, color: C.text, margin: 0,
    breakLine: false, breakLineOnOverflow: false, fit: 'shrink', valign: 'top',
  });
}

function buildMeetingOne() {
  const pptx = newDeck(
    '第一次组会：VRX环境下WAM-V自主航点导航系统设计',
    '平台搭建与基础闭环控制'
  );

  addTitleSlide(
    pptx,
    SIM_TITLE,
    '第一次组会',
    'VRX 环境下 WAM-V\n自主航点导航系统设计',
    '平台搭建与基础闭环控制',
    ['ROS 2 框架', 'Gazebo 仿真', '航点任务', '闭环控制'],
    '搭建仿真环境 → 打通控制接口 → 编写自主控制器 → 完成基础任务',
    '开场可以先说明，本项目研究的是在VRX虚拟水域中，让WAM-V无人船依靠自身传感器完成航点导航，而不是由人工遥控。VRX是无人船算法常用的仿真竞赛环境，能够提供水面、岸线、风浪、浮标以及官方任务评分。WAM-V采用左右两个推进器，两个推进器推力相同可以前进，推力不同可以转向。\n\n本次汇报重点不是介绍所有高级算法，而是说明第一阶段如何把系统真正跑起来。我将依次讲四个问题：任务给控制器什么信息，控制器需要输出什么；传感器、任务状态和推进器怎样形成闭环；第一版状态机和控制律怎样工作；当前版本还存在什么问题。\n\n这一页最后可以用一句话过渡：第一阶段的目标是先让无人船具备“能根据实时误差自主运动”的基本能力，为后面的定位融合、建图和路径规划打基础。'
  );

  {
    const slide = addContentBase(pptx, '本阶段工作一：搭建环境并打通控制接口', '第一次组会', 2);
    slide.addText('本页重点：上方是我完成的三项工程工作，下方是这些工作最终打通的控制闭环。', {
      x: 0.68, y: 1.01, w: 9.7, h: 0.2,
      fontFace: FONT, fontSize: 9.5, color: C.muted, margin: 0,
    });
    addCard(slide, pptx, 0.65, 1.22, 3.78, 1.6, C.blue, '01', '完成仿真环境搭建',
      '编译 ROS 2 Humble 与 VRX 工作区\n配置 Gazebo Garden 资源路径\n验证 WAM-V 模型、传感器与世界\n制作无竞赛倒计时学习场景');
    addCard(slide, pptx, 4.77, 1.22, 3.78, 1.6, C.teal, '02', '完成控制接口梳理',
      '定位 GPS、IMU、雷达和航点话题\n确认消息类型与服务质量策略\n测试推进器方向和推力正负号\n明确任务开始、运行与结束状态');
    addCard(slide, pptx, 8.89, 1.22, 3.78, 1.6, C.orange, '03', '编写一键启动脚本',
      '自动加载 ROS 2 与工作区环境\n等待关键话题真正收到数据\n启动仿真后自动运行控制器\n退出时安全停止并清理进程');

    slide.addText('在此基础上打通的闭环链路', {
      x: 0.68, y: 3.22, w: 3.5, h: 0.3,
      fontFace: FONT, fontSize: 13, bold: true, color: C.navy, margin: 0,
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 0.65, y: 3.61, w: 12.02, h: 2.75,
      line: { color: C.line, width: 1 }, fill: { color: C.white },
    });
    const xs = [0.93, 2.93, 4.93, 6.93, 8.93, 10.93];
    const items = [
      ['传感器', '卫星定位 / 惯导 / 雷达', C.blue],
      ['状态处理', '东-北局部坐标', C.teal],
      ['任务管理', '航点与状态机', C.orange],
      ['航点制导', '距离 / 方位 / 航向', C.green],
      ['闭环控制', '误差反馈与安全约束', C.red],
      ['WAM-V', '差动推力驱动', C.navy],
    ];
    items.forEach((item, i) => {
      addPipelineBox(slide, pptx, xs[i], 4.19, 1.6, item[0], item[1], item[2]);
      if (i < items.length - 1) addFlowArrow(slide, pptx, xs[i] + 1.68, 4.51, 0.24);
    });
    slide.addText('反馈：位置、航向、速度与障碍距离重新进入下一周期', {
      x: 2.85, y: 5.72, w: 7.65, h: 0.28,
      fontFace: FONT, fontSize: 10.5, color: C.muted, align: 'center', margin: 0,
    });
    slide.addShape(pptx.ShapeType.line, {
      x: 1.7, y: 5.55, w: 9.9, h: 0,
      line: { color: C.teal, width: 1.3, beginArrowType: 'triangle' },
    });
    slide.addNotes('这一页要明确告诉导师哪些是我在第一阶段亲自完成的工程工作。第一项是仿真环境。我完成了ROS 2 Humble与VRX工作区编译，配置Gazebo Garden的模型和世界资源路径，确认WAM-V、卫星定位、惯性测量和激光雷达都能正常生成数据。为了避免官方任务约五分钟后结束，我还制作了无竞赛倒计时的学习场景，便于长时间调试。\n\n第二项是控制接口梳理。我逐个确认了GPS、IMU、激光雷达、相机、官方航点、任务状态和左右推进器对应的话题名称、消息类型与服务质量策略，并通过低推力实验确认推进器方向和正负号。这个步骤解决了“代码写出来但接不到数据、推力方向相反、任务状态不同步”等实际问题。\n\n第三项是编写一键启动脚本。脚本自动加载ROS和工作区环境，先启动仿真，再循环检查关键话题是否存在并且真正收到消息，满足条件后才启动控制器；退出时向仿真发送中断并等待进程清理，避免残留节点继续发布推力。\n\n完成这些工作后，我才打通下方闭环。传感器数据进入状态处理，任务管理选择当前航点，航点制导计算距离和方向，反馈控制生成前进量和转向量，最后驱动WAM-V。可以总结为：这一页展示的不是VRX原本自带的功能，而是我把分散的仿真、任务和推进器接口整理成了一套可重复启动的控制实验环境。');
  }

  {
    const slide = addContentBase(pptx, '本阶段工作二：实现第一版自主控制器', '第一次组会', 3);
    slide.addText('本页重点：上方是我设计的任务状态机，左下是控制算法，右下是实际仿真结果。', {
      x: 0.68, y: 1.01, w: 10.2, h: 0.2,
      fontFace: FONT, fontSize: 9.5, color: C.muted, margin: 0,
    });
    const states = [
      ['等待', '等待任务开始', C.muted],
      ['离岸', '倒车离岸 20 米', C.blue],
      ['转向', '对准首个有效航点', C.orange],
      ['航行', '逐航点闭环导航', C.teal],
      ['对准', '调整最终艏向', C.red],
      ['完成', '停车并输出评分', C.green],
    ];
    states.forEach((state, i) => {
      const x = 0.55 + i * 2.08;
      addStateBox(slide, pptx, x, state[0], state[1], state[2]);
      if (i < states.length - 1) addFlowArrow(slide, pptx, x + 1.79, 1.79, 0.24);
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 3.72, y: 2.68, w: 5.9, h: 0.27,
      line: { color: C.line, width: 0.7 }, fill: { color: C.white },
    });
    slide.addText('代码证据：autonomous_controller.py（881 行）  |  launch_wayfinding.sh（70 行）', {
      x: 3.88, y: 2.75, w: 5.58, h: 0.13,
      fontFace: FONT, fontSize: 8.5, color: C.muted, align: 'center', margin: 0,
    });

    slide.addShape(pptx.ShapeType.rect, {
      x: 0.65, y: 3.05, w: 5.45, h: 3.45,
      line: { color: C.line, width: 1 }, fill: { color: C.white },
    });
    slide.addText('控制律与安全约束', {
      x: 0.92, y: 3.31, w: 2.5, h: 0.3,
      fontFace: FONT, fontSize: 14.5, bold: true, color: C.navy, margin: 0,
    });
    slide.addText('航向误差 eψ = 目标航向 ψd − 当前航向 ψ\n控制量 = 比例项 + 积分项 + 微分项', {
      x: 0.92, y: 3.84, w: 4.7, h: 0.85,
      fontFace: FONT, fontSize: 17, bold: true, color: C.blue, margin: 0,
    });
    slide.addText('左推力 = 前进量 − 转向量    右推力 = 前进量 + 转向量', {
      x: 0.92, y: 4.82, w: 4.7, h: 0.34,
      fontFace: FONT, fontSize: 16, bold: true, color: C.teal, margin: 0,
    });
    slide.addText('• 积分与输出限幅，避免控制量持续饱和\n• 传感器未就绪或任务结束时，推力强制归零\n• 雷达危险区停止前进，保留差动转向能力', {
      x: 0.92, y: 5.35, w: 4.65, h: 0.9,
      fontFace: FONT, fontSize: 11, color: C.text, margin: 0, breakLine: false,
    });

    slide.addImage({ path: SIM_CARD, x: 6.38, y: 3.05, w: 6.28, h: 2.45 });
    slide.addShape(pptx.ShapeType.rect, {
      x: 6.38, y: 5.5, w: 3.08, h: 1.0,
      line: { color: C.green, width: 1 }, fill: { color: 'EAF4EF' },
    });
    slide.addText('阶段成果', {
      x: 6.62, y: 5.69, w: 1.1, h: 0.25,
      fontFace: FONT, fontSize: 12, bold: true, color: C.green, margin: 0,
    });
    slide.addText('完成自动离岸、航点切换、艏向对准', {
      x: 6.62, y: 6.02, w: 2.55, h: 0.27,
      fontFace: FONT, fontSize: 9.4, color: C.text, margin: 0,
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 9.58, y: 5.5, w: 3.08, h: 1.0,
      line: { color: C.orange, width: 1 }, fill: { color: 'FBF1E7' },
    });
    slide.addText('下一步', {
      x: 9.82, y: 5.69, w: 1.0, h: 0.25,
      fontFace: FONT, fontSize: 12, bold: true, color: C.orange, margin: 0,
    });
    slide.addText('提升定位稳定性，建立障碍记忆与可行路径', {
      x: 9.82, y: 6.02, w: 2.55, h: 0.27,
      fontFace: FONT, fontSize: 9.4, color: C.text, margin: 0,
    });
    slide.addNotes('这一页展示我实际编写的第一版自主控制器。代码证据在中间：主控制器约八百八十一行，启动与进程管理脚本约七十行。控制器并不是对官方示例做简单参数修改，而是包含坐标转换、任务状态机、反馈控制、雷达避障、视觉颜色检测和安全停车逻辑。\n\n上方状态机是针对仿真实际问题设计的。系统启动后等待任务、航点和传感器数据全部就绪；出生点靠岸，因此先倒车二十米离岸；随后转向首个尚未访问的航点，再进入逐航点航行；进入捕获半径后单独调整最终艏向；完成全部目标后推进器归零。这套状态机解决了启动时撞岸、航点切换混乱和任务结束后仍持续推力的问题。\n\n左下是控制算法。我实现了经纬度到局部米制坐标转换、航向角环绕、带积分与输出限幅的反馈控制，以及左右推进器差动混合。传感器未就绪时不允许动作，雷达危险区停止前进但保留转向能力。代码还实现了基于颜色的视觉检测接口，为后续任务层预留感知能力。\n\n右下是结果。在不使用键盘遥控的情况下，控制器能够完成自动离岸、航点切换和最终艏向对准。通过这一阶段，我不仅验证了控制公式，也处理了推进器极性、状态同步、捕获半径抖动和退出清理等工程问题。');
  }

  {
    const slide = addContentBase(pptx, '阶段总结：我做了什么、解决了什么、学到了什么', '第一次组会', 4);
    slide.addText('导师视角：本页把项目已有平台与我新增的工作分开，并给出可验证成果和能力成长。', {
      x: 0.68, y: 1.02, w: 10.4, h: 0.2,
      fontFace: FONT, fontSize: 9.5, color: C.muted, margin: 0,
    });
    addContributionColumn(slide, pptx, 0.65, C.blue, '我完成了什么', '代码与工程产出', [
      '搭建并编译 ROS 2、Gazebo 与 VRX 仿真环境',
      '梳理传感器、任务与推进器接口及消息类型',
      '编写 881 行自主控制器和一键启动脚本',
      '实现坐标转换、反馈控制、雷达避障与视觉检测',
      '实现离岸、航行、对准、完成等任务状态机',
    ]);
    addContributionColumn(slide, pptx, 4.78, C.orange, '解决了什么问题', '从“能启动”到“能自主完成任务”', [
      '解决出生点靠岸导致启动碰撞的问题',
      '解决推进器极性和左右差动方向不明确的问题',
      '解决任务、航点和传感器就绪不同步的问题',
      '解决航点捕获附近反复切换和最终艏向对准问题',
      '解决退出后残留进程继续发布推力的问题',
    ]);
    addContributionColumn(slide, pptx, 8.91, C.green, '我学到了什么', '理论知识与工程能力', [
      '掌握 ROS 2 发布订阅、服务质量策略和启动系统',
      '理解坐标系、经纬度换算和四元数航向提取',
      '掌握反馈控制、抗积分饱和和差动推力分配',
      '学会用状态机组织复杂任务与安全条件',
      '形成从日志、话题和仿真现象定位问题的方法',
    ]);
    slide.addShape(pptx.ShapeType.rect, {
      x: 0.65, y: 6.12, w: 12.03, h: 0.58,
      line: { color: C.green, width: 1 }, fill: { color: 'EAF4EF' },
    });
    slide.addText('验证证据：无需键盘遥控即可完成自动离岸、航点切换与最终艏向对准；下一阶段重点解决定位噪声和环境记忆。', {
      x: 0.96, y: 6.31, w: 11.4, h: 0.22,
      fontFace: FONT, fontSize: 10.8, bold: true, color: C.green, align: 'center', margin: 0,
    });
    slide.addNotes('这一页要用个人贡献的方式总结。左侧“我完成了什么”是实际产出：从仿真环境和接口梳理开始，写出八百八十一行第一版自主控制器和启动脚本，并实现坐标转换、反馈控制、雷达避障、视觉检测、任务状态机和安全停车。这些内容都不是简单运行官方示例，而是针对WAM-V任务接口完成的独立控制链。\n\n中间“解决了什么问题”体现工程价值。我处理了出生点靠岸、推进器极性、数据与任务不同步、航点捕获抖动、最终艏向对准和进程退出残留等问题。这些问题不一定体现在算法公式里，但决定系统能否稳定运行。\n\n右侧“我学到了什么”体现能力成长。我从项目中掌握了ROS 2通信与启动机制、坐标系与四元数、反馈控制和差动推进、状态机设计，以及结合话题、日志和仿真现象排查系统问题的方法。\n\n底部验证证据说明结果不是停留在代码层面：当前控制器在无需键盘遥控的情况下能够完成自动离岸、航点切换和最终艏向对准。下一阶段将根据实际运行暴露出的定位噪声和环境记忆问题继续改进。');
  }
  return pptx;
}

function addMetricTile(slide, pptx, x, title, value, color) {
  slide.addShape(pptx.ShapeType.rect, {
    x, y: 5.75, w: 2.88, h: 0.86,
    line: { color: C.line, width: 1 }, fill: { color: C.white },
  });
  slide.addShape(pptx.ShapeType.rect, {
    x, y: 5.75, w: 0.08, h: 0.86,
    line: { color, transparency: 100 }, fill: { color },
  });
  slide.addText(title, {
    x: x + 0.22, y: 5.91, w: 1.45, h: 0.2,
    fontFace: FONT, fontSize: 9.2, color: C.muted, margin: 0,
  });
  slide.addText(value, {
    x: x + 1.5, y: 5.88, w: 1.12, h: 0.25,
    fontFace: FONT, fontSize: 11.2, bold: true, color, align: 'right', margin: 0,
  });
}

function buildMeetingTwo() {
  const pptx = newDeck(
    '第二次组会：卫星定位与惯性测量融合及滚动占据栅格环境建模',
    '状态估计、点云处理与局部地图'
  );

  addTitleSlide(
    pptx,
    LIDAR_TITLE,
    '第二次组会',
    '基于卫星定位与惯性测量融合及\n滚动占据栅格的环境建模',
    '状态估计、点云处理与局部地图',
    ['六状态融合', '点云滤波', '滚动地图', '故障回退'],
    '发现定位问题 → 实现融合估计 → 构建滚动地图 → 完成模块验证',
    '第二次汇报承接第一阶段暴露出的两个问题：原始卫星定位存在噪声和偶发跳点，单帧雷达也不能告诉规划器障碍是否持续存在。针对这些问题，我新增了状态估计、故障监督、点云处理和滚动地图模块。\n\n定位方面，我实现了六状态扩展卡尔曼滤波器、卫星定位局部里程计适配器，以及主估计器与自研估计器之间的健康检查和自动回退。环境方面，我实现了水面平面滤除、空间聚类、栅格射线更新、对数几率融合、证据衰减和障碍膨胀。\n\n本次汇报不仅说明算法原理，还会展示新增代码模块、运行指标、实际雷达截图和三十四项针对性静态测试。核心目标是证明我把第一阶段发现的问题转化成了可运行、可检查、可测试的工程模块。'
  );

  {
    const slide = addContentBase(pptx, '本阶段工作一：实现状态估计与故障回退', '第二次组会', 2);
    slide.addText('本页重点：左侧是我实现的六状态滤波器，右侧是双估计器监督，底部是运行时可检查指标。', {
      x: 0.68, y: 1.01, w: 10.4, h: 0.2,
      fontFace: FONT, fontSize: 9.5, color: C.muted, margin: 0,
    });
    slide.addText('自研六状态扩展卡尔曼滤波', {
      x: 0.66, y: 1.22, w: 4.8, h: 0.32,
      fontFace: FONT, fontSize: 15, bold: true, color: C.navy, margin: 0,
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 0.65, y: 1.67, w: 5.35, h: 0.72,
      line: { color: C.blue, width: 1.2 }, fill: { color: 'EAF2F7' },
    });
    slide.addText('x = [ E, N, vE, vN, ψ, r ]ᵀ', {
      x: 0.9, y: 1.86, w: 4.85, h: 0.3,
      fontFace: FONT, fontSize: 20, bold: true, color: C.blue, align: 'center', margin: 0,
    });
    slide.addText('E、N：东/北位置   vE、vN：东/北速度   ψ：航向   r：艏摇角速度', {
      x: 0.85, y: 2.43, w: 5.1, h: 0.2,
      fontFace: FONT, fontSize: 8.8, color: C.muted, align: 'center', margin: 0,
    });

    addPipelineBox(slide, pptx, 0.8, 2.78, 1.75, '预测', '匀速 + 匀角速度', C.blue);
    addFlowArrow(slide, pptx, 2.67, 3.1, 0.28);
    addPipelineBox(slide, pptx, 3.05, 2.78, 1.75, '异常检验', '航向环绕 + 6σ', C.orange);
    addFlowArrow(slide, pptx, 4.92, 3.1, 0.28);
    addPipelineBox(slide, pptx, 5.3, 2.78, 1.75, '协方差更新', '约瑟夫形式，保持稳定', C.teal);

    slide.addShape(pptx.ShapeType.rect, {
      x: 0.65, y: 4.28, w: 6.4, h: 0.91,
      line: { color: C.line, width: 1 }, fill: { color: C.white },
    });
    slide.addText('卫星定位：东 / 北位置与差分速度', {
      x: 0.88, y: 4.58, w: 2.7, h: 0.25,
      fontFace: FONT, fontSize: 11.5, bold: true, color: C.blue, margin: 0,
    });
    slide.addText('惯性测量：航向 ψ 与艏摇角速度 r', {
      x: 3.62, y: 4.58, w: 3.0, h: 0.25,
      fontFace: FONT, fontSize: 11.5, bold: true, color: C.teal, margin: 0,
    });

    slide.addText('双估计器监督', {
      x: 7.45, y: 1.22, w: 2.2, h: 0.32,
      fontFace: FONT, fontSize: 15, bold: true, color: C.navy, margin: 0,
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 7.45, y: 1.72, w: 2.1, h: 1.05,
      line: { color: C.green, width: 1.2 }, fill: { color: 'EAF4EF' },
    });
    slide.addText('机器人定位融合包', {
      x: 7.58, y: 1.99, w: 1.84, h: 0.25,
      fontFace: FONT, fontSize: 11.5, bold: true, color: C.green, align: 'center', margin: 0,
    });
    slide.addText('主状态源', {
      x: 7.58, y: 2.34, w: 1.84, h: 0.2,
      fontFace: FONT, fontSize: 9, color: C.muted, align: 'center', margin: 0,
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 7.45, y: 3.05, w: 2.1, h: 1.05,
      line: { color: C.blue, width: 1.2 }, fill: { color: 'EAF2F7' },
    });
    slide.addText('自研滤波器', {
      x: 7.58, y: 3.32, w: 1.84, h: 0.25,
      fontFace: FONT, fontSize: 11.5, bold: true, color: C.blue, align: 'center', margin: 0,
    });
    slide.addText('独立一致性检查', {
      x: 7.58, y: 3.67, w: 1.84, h: 0.2,
      fontFace: FONT, fontSize: 9, color: C.muted, align: 'center', margin: 0,
    });

    slide.addShape(pptx.ShapeType.chevron, {
      x: 9.87, y: 2.55, w: 0.52, h: 0.58,
      line: { color: C.orange, transparency: 100 }, fill: { color: C.orange },
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 10.62, y: 2.02, w: 2.05, h: 1.72,
      line: { color: C.orange, width: 1.2 }, fill: { color: C.white },
    });
    slide.addText('健康选择器', {
      x: 10.82, y: 2.29, w: 1.65, h: 0.28,
      fontFace: FONT, fontSize: 13, bold: true, color: C.orange, align: 'center', margin: 0,
    });
    slide.addText('超时\n协方差过大\n位置差 > 8 m', {
      x: 10.82, y: 2.75, w: 1.65, h: 0.67,
      fontFace: FONT, fontSize: 10.2, color: C.text, align: 'center', margin: 0,
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 8.15, y: 4.42, w: 4.52, h: 0.67,
      line: { color: C.teal, width: 1 }, fill: { color: 'E7F3F1' },
    });
    slide.addText('输出：统一东-北坐标位姿、速度、艏摇角速度与不确定度', {
      x: 8.37, y: 4.64, w: 4.08, h: 0.23,
      fontFace: FONT, fontSize: 10.3, bold: true, color: C.teal, align: 'center', margin: 0,
    });

    slide.addShape(pptx.ShapeType.rect, {
      x: 2.05, y: 5.3, w: 9.25, h: 0.28,
      line: { color: C.line, width: 0.7 }, fill: { color: C.white },
    });
    slide.addText('代码证据：estimator.py（300 行） | gnss_odometry.py（252 行） | robot_localization.yaml（58 行）', {
      x: 2.2, y: 5.37, w: 8.95, h: 0.13,
      fontFace: FONT, fontSize: 8.4, color: C.muted, align: 'center', margin: 0,
    });

    addMetricTile(slide, pptx, 0.65, '位置不确定度', '位置标准差', C.blue);
    addMetricTile(slide, pptx, 3.69, '估计器差异', '位置差值', C.orange);
    addMetricTile(slide, pptx, 6.73, '异常观测', '拒绝观测数', C.red);
    addMetricTile(slide, pptx, 9.77, '故障处理', '回退次数', C.green);
    slide.addNotes('这一页首先明确代码产出。我新增了三百行的六状态滤波模块、二百五十二行的卫星定位局部里程计适配器，以及机器人定位融合配置。左侧六个状态分别是东向位置、北向位置、两个方向速度、航向和艏摇角速度。预测阶段采用短时间匀速和匀角速度模型，观测阶段融合卫星定位和惯性测量。\n\n我在实现中重点处理了三个细节。第一，航向差需要角度环绕，避免正一百七十九度和负一百七十九度被误判成大误差；第二，使用六倍标准差门限拒绝明显跳点；第三，使用约瑟夫形式更新协方差，保持数值稳定并输出位置、速度和航向的不确定度。\n\n右侧是我增加的故障监督。机器人定位融合包作为主状态源，自研滤波器独立运行。健康选择器检查主估计器是否超时、协方差是否过大、两套位置是否相差超过八米，异常时切换到备用结果并累计回退次数。底部四项指标让这些行为可以被监控，而不是出现故障后只能从船体异常运动反推原因。\n\n这部分让我从“会使用现成定位节点”进一步学习到“能够实现滤波器、理解不确定度，并为定位系统设计故障回退”。');
  }

  {
    const slide = addContentBase(pptx, '本阶段工作二：实现点云处理与滚动局部地图', '第二次组会', 3);
    slide.addText('本页重点：左侧是我的实际运行截图，右侧是我实现的五步感知与地图更新链路。', {
      x: 0.68, y: 1.01, w: 9.4, h: 0.2,
      fontFace: FONT, fontSize: 9.5, color: C.muted, margin: 0,
    });
    slide.addImage({ path: LIDAR_CARD, x: 0.65, y: 1.25, w: 7.23, h: 4.82 });
    slide.addShape(pptx.ShapeType.rect, {
      x: 0.65, y: 5.5, w: 7.23, h: 0.57,
      line: { color: C.dark, transparency: 100 }, fill: { color: C.dark, transparency: 18 },
    });
    slide.addText('项目实测：WAM-V 二维激光雷达的岸线与局部障碍回波', {
      x: 0.88, y: 5.69, w: 6.75, h: 0.22,
      fontFace: FONT, fontSize: 10, color: C.white, margin: 0,
    });

    slide.addShape(pptx.ShapeType.rect, {
      x: 8.15, y: 1.25, w: 4.52, h: 4.82,
      line: { color: C.line, width: 1 }, fill: { color: C.white },
    });
    slide.addText('感知到规划的处理链', {
      x: 8.43, y: 1.52, w: 3.95, h: 0.3,
      fontFace: FONT, fontSize: 14.5, bold: true, color: C.navy, margin: 0,
    });
    slide.addText('核心代码：occupancy_grid.py / core.py / node.py', {
      x: 8.43, y: 1.84, w: 3.85, h: 0.17,
      fontFace: FONT, fontSize: 8.3, color: C.muted, margin: 0,
    });
    const steps = [
      ['1', '水面平面滤除', '去除波面与姿态扰动', C.blue],
      ['2', '空间聚类与确认', '多帧形成低浮标候选', C.teal],
      ['3', '栅格射线遍历', '自由空间与占用命中', C.orange],
      ['4', '对数几率融合与衰减', '跨帧积累，旧证据回到未知', C.green],
      ['5', '障碍膨胀快照', '提供给状态格规划器检查', C.red],
    ];
    steps.forEach((step, i) => {
      const y = 2.03 + i * 0.68;
      slide.addShape(pptx.ShapeType.rect, {
        x: 8.43, y, w: 0.44, h: 0.44,
        line: { color: step[3], transparency: 100 }, fill: { color: step[3] },
      });
      slide.addText(step[0], {
        x: 8.43, y: y + 0.1, w: 0.44, h: 0.2,
        fontFace: FONT, fontSize: 9.5, bold: true, color: C.white, align: 'center', margin: 0,
      });
      slide.addText(step[1], {
        x: 9.05, y: y + 0.01, w: 1.75, h: 0.23,
        fontFace: FONT, fontSize: 11, bold: true, color: C.navy, margin: 0,
      });
      slide.addText(step[2], {
        x: 9.05, y: y + 0.28, w: 3.25, h: 0.2,
        fontFace: FONT, fontSize: 8.7, color: C.muted, margin: 0,
      });
    });
    slide.addShape(pptx.ShapeType.rect, {
      x: 8.43, y: 5.53, w: 3.97, h: 0.38,
      line: { color: C.orange, width: 0.8 }, fill: { color: 'FBF1E7' },
    });
    slide.addText('开阔水面已有卫星定位：采用局部地图，不强行套用同步定位与建图', {
      x: 8.55, y: 5.64, w: 3.72, h: 0.17,
      fontFace: FONT, fontSize: 8.4, bold: true, color: C.orange, align: 'center', margin: 0,
    });

    slide.addShape(pptx.ShapeType.rect, {
      x: 0.65, y: 6.3, w: 12.02, h: 0.55,
      line: { color: C.line, width: 1 }, fill: { color: C.white },
    });
    const stats = [
      ['地图范围', '100 m × 100 m'],
      ['分辨率', '每格 0.5 米'],
      ['证据衰减', '8 秒后回到未知'],
      ['规划膨胀', '安全半径 3 m'],
    ];
    stats.forEach((stat, i) => {
      const x = 0.92 + i * 2.95;
      slide.addText(stat[0], {
        x, y: 6.47, w: 1.0, h: 0.18,
        fontFace: FONT, fontSize: 8.7, color: C.muted, margin: 0,
      });
      slide.addText(stat[1], {
        x: x + 1.0, y: 6.44, w: 1.7, h: 0.22,
        fontFace: FONT, fontSize: 10.5, bold: true, color: [C.blue, C.teal, C.orange, C.red][i], align: 'right', margin: 0,
      });
    });
    slide.addNotes('这一页展示我实现的第二组核心模块。代码主要位于occupancy_grid.py、core.py和node.py。左侧不是示意图，而是运行项目后截取的WAM-V实际激光雷达回波，可以看到岸线和零散障碍。\n\n右侧五个步骤对应我完成的处理链。第一步拟合水面高度平面，过滤波面和姿态扰动造成的噪点；第二步对剩余点做空间聚类和多帧确认，得到稳定低浮标候选；第三步使用栅格射线遍历，把传感器到命中点之间标记为自由空间，把终点附近标记为占用；第四步用对数几率融合多帧证据，并让八秒未更新的旧证据衰减回未知；第五步在规划快照中膨胀障碍，为船体宽度和转弯扫掠留出三米安全半径。\n\n我还实现了地图随船移动时的整格重定位、异常量程处理、命中优先和同帧单格更新，避免相邻光束重复清除低浮标。地图最终发布为ROS占据栅格，并在RViz中与雷达回波对齐显示。\n\n这一部分让我学习了点云几何处理、逆传感器模型、对数几率地图、时间衰减和坐标变换。也让我理解了算法选择要结合场景：当前开阔水面已有卫星定位，因此局部滚动地图比强行保存长期同步定位与建图结果更合适。');
  }

  {
    const slide = addContentBase(pptx, '阶段总结：新增模块、验证证据与学习收获', '第二次组会', 4);
    slide.addText('导师视角：本页用代码模块、针对性测试和运行截图说明本阶段确实完成了可验证增量。', {
      x: 0.68, y: 1.02, w: 10.4, h: 0.2,
      fontFace: FONT, fontSize: 9.5, color: C.muted, margin: 0,
    });
    addContributionColumn(slide, pptx, 0.65, C.blue, '我新增的模块', '从原始传感器到可信状态与地图', [
      '六状态扩展卡尔曼滤波与异常观测拒绝',
      '卫星定位局部里程计适配与速度观测',
      '主估计器健康检查和自动故障回退',
      '水面滤除、点云聚类和低浮标确认',
      '滚动占据栅格、证据衰减与障碍膨胀',
    ]);
    addContributionColumn(slide, pptx, 4.78, C.orange, '我如何验证', '测试、指标与实际运行证据', [
      '估计器、定位适配、地图与可视化共 34 项测试通过',
      '状态输出包含位置、速度、航向及标准差',
      '运行中记录拒绝观测数、位置差值和回退次数',
      'RViz 实际显示岸线回波、地图和浮标候选',
      '任务重启后估计器、轨迹与地图能够同步清空',
    ]);
    addContributionColumn(slide, pptx, 8.91, C.green, '我学到了什么', '算法理解与系统设计能力', [
      '理解卡尔曼滤波预测、更新与协方差含义',
      '掌握多传感器时间、坐标和不确定度融合',
      '掌握逆传感器模型、射线更新与对数几率地图',
      '学会为定位系统设计健康监督和降级策略',
      '能够根据水面场景判断是否需要长期建图',
    ]);
    slide.addShape(pptx.ShapeType.rect, {
      x: 0.65, y: 6.12, w: 12.03, h: 0.58,
      line: { color: C.green, width: 1 }, fill: { color: 'EAF4EF' },
    });
    slide.addText('阶段结论：已把“定位噪声”和“单帧雷达”问题转化为可运行、可监控、可测试的状态估计与局部地图模块。', {
      x: 0.96, y: 6.31, w: 11.4, h: 0.22,
      fontFace: FONT, fontSize: 10.8, bold: true, color: C.green, align: 'center', margin: 0,
    });
    slide.addNotes('这一页从产出、验证和学习三个方面总结第二阶段。左侧是新增模块：六状态扩展卡尔曼滤波、卫星定位局部里程计适配、主估计器健康监督和回退、水面与点云处理、滚动占据栅格、证据衰减及障碍膨胀。这些模块共同把原始传感器数据转化成带不确定度的船体状态和可供规划查询的局部地图。\n\n中间是验证证据。与本阶段直接相关的估计器、定位适配、占据栅格、调试可视化和节点辅助测试共三十四项，当前全部通过。系统运行状态还持续发布位置标准差、估计器位置差、异常观测拒绝数和回退次数；RViz截图证明雷达回波和局部环境能够正确显示；任务重新开始时，估计器、点云轨迹和地图会一起清空，避免历史数据污染新任务。\n\n右侧是学习收获。我不仅理解了卡尔曼滤波公式，还真正处理了航向环绕、协方差稳定性、异常门控和故障降级；在地图方面掌握了逆传感器模型、栅格射线、对数几率、多帧融合和时间衰减；同时学会根据开阔水面已有卫星定位这一条件，选择局部地图而不是机械套用长期同步定位与建图。\n\n底部结论强调本阶段的增量：第一阶段发现的定位噪声和单帧雷达问题，已经被转化为可运行、可监控、可测试的工程模块。下一阶段将在这些模块基础上实现曲率约束路径规划和路径跟踪。');
  }
  return pptx;
}

async function main() {
  await prepareAssets();
  const meetingOne = buildMeetingOne();
  const meetingTwo = buildMeetingTwo();
  await meetingOne.writeFile({ fileName: path.join(OUT, '第一次组会汇报_平台与基础闭环.pptx') });
  await meetingTwo.writeFile({ fileName: path.join(OUT, '第二次组会汇报_状态估计与局部地图.pptx') });
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
