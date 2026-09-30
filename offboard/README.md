# offboard/position_offboard_test

출처: [JOCIIIII/PX4-Offboard-Tests](https://github.com/JOCIIIII/PX4-Offboard-Tests) (commit 648d9fd) 의
`position_offboard_test` 패키지 (MIT, package.xml 기준).

이 레포에서 추가·변경한 것:
- `position_offboard_test/pp_waypoint_offboard.py` — PP 경유점(`config/pp_goals_ned.csv`)을 따라 나는 노드 (신규)
- `setup.py` — `pp_waypoint_offboard` 실행 파일 등록, `config/` 설치
- `config/` — `../run_pp.sh` 가 PP 결과를 자동으로 저장하는 곳

원래 예제 `offboard_control.py` 는 수정하지 않았다.
