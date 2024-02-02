import matplotlib.pyplot as plt
import numpy as np

def quat_to_rotation(quat: np.array) -> np.array:
    R = np.zeros((3,3))
    R[0,0] = 2 * (quat[0]**2 + quat[1]**2) - 1
    R[1,1] = 2 * (quat[0]**2 + quat[2]**2) - 1
    R[2,2] = 2 * (quat[0]**2 + quat[3]**2) - 1

    R[0,1] = 2 * (quat[1]*quat[2] - quat[0]*quat[3])
    R[0,2] = 2 * (quat[1]*quat[3] + quat[0]*quat[2])
    R[1,0] = 2 * (quat[1]*quat[2] + quat[0]*quat[3])
    R[1,2] = 2 * (quat[2]*quat[3] - quat[0]*quat[1])
    R[2,0] = 2 * (quat[1]*quat[3] - quat[0]*quat[2])
    R[2,1] = 2 * (quat[2]*quat[3] + quat[0]*quat[1])

    return R

def read_samples(filename: str) -> [np.array, np.array, np.array]:
    data = np.genfromtxt(filename, delimiter=",")
    translation = data[:, :3]
    rotation = data[:, 3:7]
    samples = data[:, 7:]

    return translation, rotation, samples

def main():
    ax = plt.figure().add_subplot(projection='3d')
    translation, rotation, samples = read_samples("recording.csv")

    ax.scatter(translation[:, 0], translation[:, 1], translation[:, 2], 'k')

    matrixes = list(map(quat_to_rotation, rotation))
    for i, R in enumerate(matrixes):
        offset_0 = R[:,0]
        ax.plot(
            [translation[i, 0], translation[i, 0] + offset_0[0]], 
            [translation[i, 1], translation[i, 1] + offset_0[1]], 
            [translation[i, 2], translation[i, 2] + offset_0[2]],
            'r')
        offset_1 = R[:,1]
        ax.plot(
            [translation[i, 0], translation[i, 0] + offset_1[0]], 
            [translation[i, 1], translation[i, 1] + offset_1[1]], 
            [translation[i, 2], translation[i, 2] + offset_1[2]],
            'b')
        offset_2 = R[:,2]
        ax.plot(
            [translation[i, 0], translation[i, 0] + offset_2[0]], 
            [translation[i, 1], translation[i, 1] + offset_2[1]], 
            [translation[i, 2], translation[i, 2] + offset_2[2]],
            'g')

    ax.set_xlim(min(translation[:, 0]), max(translation[:, 0]))
    ax.set_ylim(min(translation[:, 1]), max(translation[:, 1]))
    ax.set_zlim(min(translation[:, 2]), max(translation[:, 2]))

    plt.show()

if __name__ == "__main__":
    main()