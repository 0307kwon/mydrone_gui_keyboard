mydrone_gui_keyboard
========================

1.summary
---------------
드론을 키보드로 조종하거나 
timestamp 마다 position이 기록된 text를 불러들여 드론을 조종합니다.
(2020 KROC 참여)


2.how to use
-----------------
1. catkin_ws/src 폴더로 진입
2. git clone https://github.com/0307kwon/mydrone_gui_keyboard.git
3. catkin_make
4. roslaunch mydrone_gui_keyboard mydrone.launch
(tracking 관련 txt파일 위치는 launch파일에서 수정가능)

### 2-1 키보드 사용법
w : 전진
s : 후진
a : 좌로 이동
d : 우로 이동

↑ : 고도 상승
↓ : 고도 하강
( 키입력이 제대로 작동하지 않을 때에는 화면의 오른쪽 Command Panel에 focus를 두고(마우스 클릭) 다시 시도 )

![Alt text](/image/keyboard.gif "keyboard")


### 2-2 tracking 사용법
txt 파일에 저장된 position을 0.1s 마다 한줄씩 읽어 해당 위치로 가도록 명령합니다.

*txt파일 read/write 경로 수정 방법
src/mydrone_gui_keyboard/launch/mydrone.launch의 
txt_location 값을 수정해줌으로써 변경가능합니다.

*start recording 
txt_location 위치에 txt파일로 0.1s 마다 위치와 방향 등을 기록합니다.

*start tracking
txt_location 위치의 txt파일을 0.1s 마다 한줄씩 읽어와 드론에게 위치를 전송합니다.

![Alt text](/image/tracking.gif "tracking")


