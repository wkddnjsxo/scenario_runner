[save_ego_pose.py (line 1)](/home/jang/scenario_runner/tools/save_ego_pose.py:1)
현재 MORAI 시뮬레이터의 Ego 위치/자세를 gRPC로 읽어서 saved_ego_positions.yaml에 저장합니다. 동시에 현재 Ego가 올라간 링크를 후보 링크 그룹에 추가할 수 있고, PNG 미리보기까지 만듭니다.

[spawn_saved_ego_poses.py (line 1)](/home/jang/scenario_runner/tools/spawn_saved_ego_poses.py:1)
저장된 Ego pose들을 YAML 순서대로 MORAI에 다시 spawn/teleport해서, 저장된 위치들이 실제로 쓸 만한지 검증하는 도구입니다.

save_ego_pose.py 사용법
1. 모라이 시뮬레이터에서 ego차량을 저장할 스폰지점에 위치시킴(기어 p단으로 정지 추천)
2. 코드 실행
3. 해당 도로 차선에 맞게 번호 선택
4. 맞게 했다면 Y입력
5. 해당 위치 저장

시나리오 자동생성기 실행
cd /home/jang/scenario_runner

 ./run.sh

