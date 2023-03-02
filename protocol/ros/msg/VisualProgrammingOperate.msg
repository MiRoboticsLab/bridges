# 用于可视化编程项目包
# 详情参见：https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#S9ebnr

string TYPE_TASK="task"                     # 任务
string TYPE_MODULE="module"                 # 模块
string TYPE_AI="AI"                         # 人工智能（Artificial intelligence）
string TYPE_SLAM="SLAM"                     # 同步定位与地图构建（Simultaneous localization and mapping）

string MODE_FACTORY="factory"               # 厂家
string MODE_USER="user"                     # 用户

string MODE_SINGLE="single"                 # 单次
string MODE_CYCLE="cycle"                   # 周期

string MODE_COMMON="common"                 # 普通
string MODE_SEQUENCE="sequence"             # 序列

string MODE_SINGLE_STEP="single_step"       # 单步
string MODE_ORIGINAL="original"             # 原始

string MODE_ALL="all"                       # 所有

string MODE_PERSONNEL="personnel"           # 人员
string MODE_FACE="face"                     # 人脸
string MODE_VOICEPRINT="voiceprint"         # 声纹
string MODE_TAAINING_WORDS="training_words" # 训练词

string MODE_MAP="map"                       # 地图
string MODE_PRESET="preset"                 # 预置点

string OPERATE_INQUIRY="inquiry"            # [操作]查询
string OPERATE_SAVE="save"                  # [操作]保存
string OPERATE_DELETE="delete"              # [操作]删除
string OPERATE_DEBUG="debug"                # [操作]调试
string OPERATE_RUN="run"                    # [操作]运行
string OPERATE_SUSPEND="suspend"            # [操作]暂停
string OPERATE_RECOVER="recover"            # [操作]继续
string OPERATE_SHUTDOWN="shutdown"          # [操作]终止
string OPERATE_START="start"                # [操作]开始
string OPERATE_STOP="stop"                  # [操作]停止

string STATE_NULL="null"                    # [状态]空
string STATE_ERROR="error"                  # [状态]错误
string STATE_WAIT_RUN="wait_run"            # [状态]等待运行
string STATE_RUN_WAIT="run_wait"            # [状态]运行等待
string STATE_RUN="run"                      # [状态]运行
string STATE_SUSPEND="suspend"              # [状态]暂停
string STATE_SHUTDOWN="shutdown"            # [状态]终止
string STATE_NORMAL="normal"                # [状态]正常

string STEP_PARSE="parse"                   # [步骤]解析
string STEP_REGISTER="register"             # [步骤]注册

string STEP_BEGIN="begin"                   # [步骤]开始
string STEP_END="end"                       # [步骤]结束

string FSM_UNINITIALIZED="Uninitialized"    # [状态机] Uninitialized
string FSM_SET_UP="SetUp"                   # [状态机] SetUp
string FSM_TEAR_DOWN="TearDown"             # [状态机] TearDown
string FSM_SELF_CHECK="SelfCheck"           # [状态机] SelfCheck
string FSM_ACTIVE="Active"                  # [状态机] Active
string FSM_DE_ACTIVE="DeActive"             # [状态机] DeActive
string FSM_PROTECTED="Protected"            # [状态机] Protected
string FSM_LOW_POWER="LowPower"             # [状态机] LowPower
string FSM_OTA="OTA"                        # [状态机] OTA
string FSM_ERROR="Error"                    # [状态机] Error

string type                                 # 消息类型
string id                                   # 消息ID
string[] target_id                          # 目标ID
string describe                             # 消息描述
string style                                # 消息样式
string operate                              # 操作类型
string mode                                 # 模式类型
string condition                            # 约束信息
string body                                 # 消息主体
string fsm                                  # 状态机状态
string[] dependent                          # 依赖项
string[] be_depended                        # 被依赖项

