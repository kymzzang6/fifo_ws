std_msgs/Header header 
# 탐지된 객체들의 클래스 이름 리스트 (예: ["helmet", "gloves"])
string[] classes

# 탐지된 객체들의 중심 좌표 (Pixel 단위)
float32[] center_xs
float32[] center_ys

# 신뢰도 점수 - 추후 필터링에 유용함
float32[] confidences
