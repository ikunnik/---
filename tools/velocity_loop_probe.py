"""D 测试：验证底层 sequence.loop=True 是否实际循环、cancel 是否能及时停车。
此脚本不会关闭机器人内置避障。必须有人监护、准备实体急停，先用其他静止安全测试验证。
运行 3 秒后主动取消、再补发零速；单纯 cancel 不被视为成功停车。
"""
import argparse
import threading
import time

import grpc
from dobot_quad.robot_client import RobotClient
from dobot_quad.proto import grpc_service_pb2 as pb2
from dobot_quad.proto.grpc_service_pb2_grpc import gRPCServiceStub


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='192.168.5.2:50051')
    args = ap.parse_args()
    print('危险提示：实验性 loop=True 尚未经实机验证；必须空旷场地、急停可触达、专人监护。')
    if input('输入 LOOP 才进行 3 秒有监护测试：').strip() != 'LOOP':
        return
    client = RobotClient(args.ip)
    channel = grpc.insecure_channel(args.ip)
    call = None
    try:
        result = client.balance_stand()
        if hasattr(result,'success') and not result.success:
            raise RuntimeError('起立失败')
        result = client.walk()
        if hasattr(result,'success') and not result.success:
            raise RuntimeError('进入行走模式失败')
        req = pb2.ExecuteSequenceRequest()
        req.immediate_start = True
        seq = req.sequence
        sid = f'loop_probe_{time.time_ns()}'
        seq.sequence_id = sid
        seq.sequence_name = sid
        seq.loop = True
        motion = seq.motions.add()
        motion.motion_id = 'walk_velocity_seq'
        param = motion.parameters.add()
        param.key = 'velocity_sequence'
        param.string_value = '0.4,0,0,1.2;0.4,0,0,1.2'
        call = gRPCServiceStub(channel).ExecuteSequence(req)
        outcome = []
        def consume():
            try:
                for progress in call:
                    if progress.is_final:
                        outcome.append(('final', bool(progress.success), time.monotonic()))
                        break
            except grpc.RpcError as e:
                outcome.append(('rpc', str(e.code()), time.monotonic()))
        thread = threading.Thread(target=consume, daemon=True)
        start = time.monotonic()
        thread.start()
        time.sleep(3.0)
        print('到达 3 秒：尝试 cancel + 发送零速。')
        call.cancel()
        # 不阻塞在取消回执上：立即单独发送零速
        client.velocity_sequence('0,0,0,0.5', gait='walk', stand_down_after=False,
                                 show_progress=False)
        thread.join(timeout=1.0)
        print('结果：', outcome, '；经过', round(time.monotonic()-start,2),'秒')
        print('请人工确认是否在 2.4 秒时停顿、是否循环、3 秒取消后是否及时停车。')
    finally:
        if call is not None:
            call.cancel()
        try:
            client.velocity_sequence('0,0,0,0.5', gait='walk',
                                     stand_down_after=False, show_progress=False)
        except Exception as err:
            print('警告：补发零速失败，请使用实体急停：',err)
        channel.close()
        try:
            client.channel.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
