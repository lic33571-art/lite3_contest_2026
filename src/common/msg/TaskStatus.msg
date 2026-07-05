# 任务状态反馈
# current_task: 当前任务名称
# current_state: 当前状态描述
# progress: 进度百分比 0~100

string current_task       # "INIT" / "LINE_TRACKING" / "INSPECTION" / "GRASP_TASK" / "COMPLETE"
string current_state      # 状态描述
int32 progress            # 进度百分比
