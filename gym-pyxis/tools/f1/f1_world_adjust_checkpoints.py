import json
import curses
import argparse
from curses import wrapper
from gym_pyxis.envs.gazebo.f1 import F1

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion


def get_checkpoint(checkpoint_id, checkpoints):
    checkpoint_id = checkpoint_id % len(checkpoints)
    if str(checkpoint_id) in checkpoints:
        checkpoint = checkpoints[str(checkpoint_id)]
        x = checkpoint[0]
        y = checkpoint[1]
        z = checkpoint[2]
        roll = checkpoint[3]
        pitch = checkpoint[4]
        yaw = checkpoint[5]
        return checkpoint_id, x, y, z, roll, pitch, yaw
    return None


def main(stdscr, checkpoints):
    stdscr.clear()
    stdscr.refresh()

    model_name = 'f1_renault'
    f1 = F1()
    odometry = f1.get_odometry()
    original_pose = odometry.pose.pose
    euler_orientation = euler_from_quaternion((original_pose.orientation.x, original_pose.orientation.y,
                                          original_pose.orientation.z, original_pose.orientation.w))

    for checkpoint_id, checkpoint in checkpoints.items():
        checkpoints[checkpoint_id] = [checkpoint[0], checkpoint[1], checkpoint[2],
                                      euler_orientation[0], euler_orientation[1], euler_orientation[2]]

    checkpoint_id = 0
    checkpoint_id, x, y, z, roll, pitch, yaw = get_checkpoint(checkpoint_id, checkpoints)

    k = 0
    linear_increment = 0.05
    angular_increment = 0.01

    while k != ord('q'):
        stdscr.clear()

        if k == curses.KEY_DOWN:
            x -= linear_increment
        elif k == curses.KEY_UP:
            x += linear_increment
        elif k == curses.KEY_LEFT:
            y -= linear_increment
        elif k == curses.KEY_RIGHT:
            y += linear_increment
        elif k == ord('z'):
            z += linear_increment
        elif k == ord('x'):
            z -= linear_increment
        elif k == ord('a'):
            yaw += angular_increment
        elif k == ord('s'):
            yaw -= angular_increment
        elif k == ord('n'):
            checkpoint_id += 1
            checkpoint = get_checkpoint(checkpoint_id, checkpoints)
            if checkpoint is not None:
                checkpoint_id, x, y, z, roll, pitch, yaw = checkpoint
            else:
                stdscr.addstr('Checkpoint with id {} doenst exists\n'.format(checkpoint_id))
        elif k == ord('p'):
            checkpoint_id -= 1
            checkpoint = get_checkpoint(checkpoint_id, checkpoints)
            if checkpoint is not None:
                checkpoint_id, x, y, z, roll, pitch, yaw = checkpoint
            else:
                stdscr.addstr('Checkpoint with id {} doenst exists\n'.format(checkpoint_id))
        elif k == ord('k'):
            stdscr.addstr('Checkpoint {} stored\n'.format(checkpoint_id))
            checkpoints[str(checkpoint_id)] = [x, y, z, roll, pitch, yaw]
            if checkpoint_id < len(checkpoints) - 1:
                checkpoint_id += 1
                checkpoint = get_checkpoint(checkpoint_id, checkpoints)
                if checkpoint is not None:
                    checkpoint_id, x, y, z, roll, pitch, yaw = checkpoint
                    stdscr.addstr('New checkpoint id {}\n'.format(checkpoint_id))
                else:
                    stdscr.addstr('No more checkpoints\n')

        stdscr.addstr('id:{}, x:{}, y:{}, z:{}, roll:{}, pitch:{}, yaw:{}\n'.format(checkpoint_id, x, y, z,
                                                                                    roll, pitch, yaw))
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        f1.set_pose(x, y, z, quaternion[0], quaternion[1], quaternion[2], quaternion[3], model_name=model_name)
        stdscr.refresh()
        k = stdscr.getch()

    stdscr.getch()

    for checkpoint_id, checkpoint in checkpoints.items():
        quaternion = quaternion_from_euler(checkpoint[3], checkpoint[4], checkpoint[5])
        checkpoints[checkpoint_id] = [checkpoint[0], checkpoint[1], checkpoint[2],
                                      quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

    with open('adjusted_checkpoints.json', 'w') as fd:
        json.dump(checkpoints, fd, indent=4)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoints",  type=str, help="Checkpoints filepath", required=True)
    args = parser.parse_args()

    with open(args.checkpoints, 'r') as fd:
        checkpoints = json.load(fd)

    wrapper(main, checkpoints)



