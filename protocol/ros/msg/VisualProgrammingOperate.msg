# 用于可视化编程项目包
# 详情参见：https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#S9ebnr

string TYPE_TASK="task"                 # 任务
string TYPE_MODULE="module"             # 模块

string MODE_FACTORY="factory"           # 厂家
string MODE_USER="user"                 # 用户

string MODE_SINGLE="single"             # 单次
string MODE_CYCLE="cycle"               # 周期

string MODE_COMMON="common"             # 普通
string MODE_SEQUENCE="sequence"         # 序列

string MODE_SINGLE_STEP="single_step"   # 单步
string MODE_ORIGINAL="original"         # 原始

string OPERATE_SAVE="save"              # [操作]保存
string OPERATE_INQUIRY="inquiry"        # [操作]查询
string OPERATE_DELETE="delete"          # [操作]删除
string OPERATE_DEBUG="debug"            # [操作]调试
string OPERATE_RUN="run"                # [操作]运行
string OPERATE_SHUTDOWN="shutdown"      # [操作]终止
string OPERATE_SUSPEND="suspend"        # [操作]暂停
string OPERATE_RECOVER="recover"        # [操作]继续
string OPERATE_START="start"            # [操作]开始
string OPERATE_STOP="stop"              # [操作]停止

string STATE_NULL="null"                # [状态]空
string STATE_ERROR="error"              # [状态]错误
string STATE_WAIT_RUN="wait_run"        # [状态]等待运行
string STATE_RUN_WAIT="run_wait"        # [状态]运行等待
string STATE_RUN="run"                  # [状态]运行
string STATE_SUSPEND="suspend"          # [状态]暂停
string STATE_SHUTDOWN="shutdown"        # [状态]终止
string STATE_NORMAL="normal"            # [状态]正常

string STEP_PARSE="parse"               # [步骤]解析
string STEP_REGISTER="register"         # [步骤]注册

string STEP_BEGIN="begin"               # [步骤]开始
string STEP_END="end"                   # [步骤]结束

string type
string id
string[] target_id
string describe
string style
string operate
string mode
string condition
string body
