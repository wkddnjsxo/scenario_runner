#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BEV 토픽 데이터 수집 및 저장
===========================
/bev/lane, /bev/obstacle, /bev/global_path 를 구독하고
/root/aim_ws/data/bev_map/ 에 저장

파일명 형식: map_<timestamp>_<loop_count>.npz
"""

import os
import sys
import time
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32
from datetime import datetime

# ======================================================================
# 설정
# ======================================================================
DATA_DIR = "/root/aim_ws/data/bev_map"

class BEVDataCollector:
    def __init__(self):
        rospy.init_node('bev_data_collector', anonymous=True)
        
        # 데이터 저장소
        self.lane_data = None
        self.obstacle_data = None
        self.path_data = None
        self.timestamp = None
        self.reset_count = 0
        self.current_scenario_dir = None
        
        # 구독자 설정
        rospy.Subscriber('/bev/lane', OccupancyGrid, self.lane_callback)
        rospy.Subscriber('/bev/obstacle', OccupancyGrid, self.obstacle_callback)
        rospy.Subscriber('/bev/global_path', OccupancyGrid, self.path_callback)
        rospy.Subscriber('/reset_count', Int32, self.reset_count_callback)
        
        print("✓ BEV 토픽 구독 시작")
        print(f"✓ 저장 기본 경로: {DATA_DIR}")
    
    # ====================================================================
    # 콜백 함수
    # ====================================================================
    
    def lane_callback(self, msg):
        """차선 데이터 콜백"""
        self.lane_data = self._occupancy_grid_to_array(msg)
        self.timestamp = time.time()
    
    def obstacle_callback(self, msg):
        """장애물 데이터 콜백"""
        self.obstacle_data = self._occupancy_grid_to_array(msg)
        self.timestamp = time.time()
    
    def path_callback(self, msg):
        """경로 데이터 콜백"""
        self.path_data = self._occupancy_grid_to_array(msg)
        self.timestamp = time.time()
    
    def reset_count_callback(self, msg):
        """reset_count 콜백 - 새로운 시나리오 디렉토리 생성"""
        new_reset_count = msg.data
        if new_reset_count != self.reset_count:
            self.reset_count = new_reset_count
            # 시나리오 디렉토리 생성
            self.current_scenario_dir = os.path.join(DATA_DIR, f"scenario_{self.reset_count:03d}")
            os.makedirs(self.current_scenario_dir, exist_ok=True)
            print(f"\n📂 시나리오 디렉토리 생성: scenario_{self.reset_count:03d}")
    
    # ====================================================================
    # 유틸리티 함수
    # ====================================================================
    
    def _occupancy_grid_to_array(self, msg):
        """OccupancyGrid 메시지를 numpy 배열로 변환"""
        arr = np.array(msg.data, dtype=np.uint8)
        arr = arr.reshape((msg.info.height, msg.info.width))
        return arr
    
    def save_bev_data(self):
        """수집된 BEV 데이터를 npz 파일로 저장"""
        if self.lane_data is None or self.obstacle_data is None or self.path_data is None:
            return False
        
        # 시나리오 디렉토리가 없으면 기본 경로 사용
        if self.current_scenario_dir is None:
            save_dir = DATA_DIR
        else:
            save_dir = self.current_scenario_dir
        
        os.makedirs(save_dir, exist_ok=True)
        
        # 타임스탬프 생성
        ts = datetime.fromtimestamp(self.timestamp).strftime('%Y%m%d_%H%M%S')
        
        # 파일명: map_<timestamp>.npz
        filename = f"map_{ts}.npz"
        filepath = os.path.join(save_dir, filename)
        
        # 데이터 저장
        np.savez_compressed(
            filepath,
            lane=self.lane_data,
            obstacle=self.obstacle_data,
            path=self.path_data,
            timestamp=np.float64(self.timestamp),
            reset_count=np.int32(self.reset_count)
        )
        
        print(f"✓ 저장: scenario_{self.reset_count:03d}/map_{ts}.npz")
        
        return True
    
    def load_bev_data(self, filename):
        """npz 파일에서 BEV 데이터 로드"""
        filepath = os.path.join(DATA_DIR, filename)
        if not os.path.exists(filepath):
            print(f"✗ 파일을 찾을 수 없습니다: {filepath}")
            return None
        
        data = np.load(filepath)
        return {
            'lane': data['lane'],
            'obstacle': data['obstacle'],
            'path': data['path'],
            'timestamp': float(data['timestamp']),
            'reset_count': int(data['reset_count'])
        }
    
    def run(self):
        """메인 루프"""
        print("\n" + "="*60)
        print("🚗 BEV 데이터 수집 시작")
        print("="*60)
        print("매초마다 BEV 데이터를 수집하고 저장합니다.")
        print("종료: Ctrl+C\n")
        
        last_save_time = time.time()
        save_interval = 1.0  # 1초마다 저장
        
        try:
            while not rospy.is_shutdown():
                now = time.time()
                
                # 1초 주기로 데이터 저장
                if now - last_save_time >= save_interval:
                    self.save_bev_data()
                    last_save_time = now
                
                rospy.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n\n⏹️  데이터 수집 중단 (Ctrl+C)")
        finally:
            print("프로그램 종료")


if __name__ == '__main__':
    try:
        collector = BEVDataCollector()
        collector.run()
    except Exception as e:
        print(f"✗ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
