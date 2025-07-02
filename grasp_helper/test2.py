import ctypes
import os


def Set_Gripper_Release_65B(pDll, nSocket, speed=200, block=1):
    ret = pDll.Set_Gripper_Release(nSocket, speed, block)
    return ret


def gripper_f_pick_65B(pDll, nSocket, force, speed=1000):
    pDll.Set_Gripper_Pick.argtypes = (ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_bool)
    pDll.Set_Gripper_Pick.restype = ctypes.c_int
    rev = pDll.Set_Gripper_Pick(nSocket, speed, force, 1)
    return rev


def main():
    CUR_PATH = os.path.dirname(__file__)
    dllPath = os.path.join(CUR_PATH, "./libRM_Based.so.1.0.0")

    pDll = ctypes.cdll.LoadLibrary(dllPath)
    dev_mode = 65
    pDll.RM_API_Init(dev_mode, 0)
    HOST_fi = '192.168.1.18'
    TCPbyte = bytes(HOST_fi, "gbk")
    nSocket = pDll.Arm_Socket_Start(TCPbyte, 8080, 2000)

    # --------------------------- >> 松开夹爪----------------------------
    #Set_Gripper_Release_65B(pDll, nSocket, speed=600, block=1)

    # --------------------------- >> 夹紧夹爪----------------------------
    gripper_f_pick_65B(pDll, nSocket, force=600, speed=200)


if __name__ == '__main__':
    main()

