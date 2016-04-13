#!/usr/bin/env python

import rospy
import numpy
from quad_control.srv import TrajDes_Srv
from quad_control.srv import PlannerStart
from std_srvs.srv import Empty


class CommandPlanner:
    def __init__(self):
        rospy.init_node('command_planner')

        # start services
        rospy.Service('planner_start', PlannerStart, self.set_and_run)
        rospy.Service('planner_stop', Empty, self.stop)

    # callback for service planner_start
    def set_and_run(self, req):
        self.set_plan(req.plan, req.default_ns)
        self.start()

        return []

    # sets a plan that is read from plan_string
    def set_plan(self, plan_string, default_ns):
        # split string into lines
        command_strings = plan_string.split('\n')

        # initialize empty command list
        self._commands = []

        # iterate over lines
        for line in command_strings:
            # split time from command part
            split_string = line.split(':', 1)
            # skip line if format does not match
            if len(split_string) < 2:
                continue


            # split time and namespace
            time_ns = split_string[0].strip().split(' ')
            # drop empty strings caused by duplicate blanks
            time_ns = filter(None, time_ns)
            # skip line if time is not a number
            if len(time_ns) >= 1:
                try:
                    time = float(time_ns[0])
                except ValueError:
                    continue

            # if namespace is specified in line read it, otherwise set to default
            if len(time_ns) >= 2:
                ns = time_ns[1]
            else:
                ns = default_ns

            # make sure namespace end with /
            if not ns.endswith('/'):
                ns = ns + '/'

            # split trajectory and controller command
            command_pair = split_string[1].split(';', 1)
            traj_command = command_pair[0]
            if len(command_pair) >= 2:
                ctrl_command = command_pair[1]
            else:
                # controller command is set to empty string if nothing is there
                ctrl_command = ''

            # append to command list
            self._commands.append([time, ns, traj_command, ctrl_command])

        # sort command list by time
        self._commands.sort()

    # start execution
    def start(self):
        self.stop()

        self._next_entry = 0
        self._start_time = rospy.get_time()

        self._curr_command = self._next()
        # iterate until next command is at time Inf

        if self._curr_command[0] < float('Inf'):
            dur = rospy.Duration((self._start_time + self._curr_command[0]) - rospy.get_time())
            self._timer = rospy.Timer(dur, self._timer_callback, True)

    # stop execution of plan (argument req an return value is for compatiblity with service callback format)
    def stop(self, req=None):
        if hasattr(self, '_timer'):
            self._timer.shutdown()

        return []

    # callback for timer (starts new timer if last command is not reached)
    def _timer_callback(self, event):
        ns = self._curr_command[1]

        traj_command = self._curr_command[2].strip()
        if len(traj_command) > 0:
            self._set_traj(traj_command, ns)

        ctrl_command = self._curr_command[3].strip()
        if len(ctrl_command) > 0:
            self._set_ctrl(ctrl_command, ns)

        # load next command
        self._curr_command = self._next()
        if self._curr_command[0] < float('Inf'):
            dur = rospy.Duration((self._start_time + self._curr_command[0]) - rospy.get_time())
            self._timer = rospy.Timer(dur, self._timer_callback, True)

    # return next entry in list (as [time,traj_command,ctrl_command])
    def _next(self):
        # at end of list return empty command with time Inf
        if self._next_entry >= len(self._commands):
            return float('Inf'), '', ''

        # return next value and increase counter
        ret_val = self._commands[self._next_entry]
        self._next_entry = self._next_entry + 1
        return ret_val

    def _set_traj(self, traj_command, ns):
        rospy.loginfo('trajectory command: \'' + traj_command + '\' on namespace ' + ns)

        try:
            rospy.wait_for_service(ns + 'TrajDes_GUI', 1.0)
            traj_service = rospy.ServiceProxy(ns + 'TrajDes_GUI', TrajDes_Srv)

            # call function
            try:
                traj, offset, rotation, parameters = eval("self._traj_" + traj_command)
                traj_service(traj, offset, rotation, parameters)
            except:
                rospy.logwarn('invalid trajectory command: '+traj_command)

        except rospy.ROSException, rospy.ServiceException:
            rospy.logerr('service to set trajectory command \'' + traj_command +
                         '\' on namespace ' +ns + ' failed.')

    def _set_ctrl(self, ctrl_command, ns):
        rospy.logwarn('namespace: ' + ns + 'controller command: ' + ctrl_command)

    # this is called for trajectory command hold()
    def _traj_hold(self, x=0, y=0, z=.5):
        traj = 0
        offset = numpy.array([x, y, z])
        rotation = numpy.array([0.0, 0.0, 0.0])
        parameters = None

        return traj, offset, rotation, parameters

    # this is called for trajectory command circle()
    def _traj_circle(self, center_x=0, center_y=0, center_z=.5, r=.5, w=1. / 2 / numpy.pi, theta=0, psi=0):
        traj = 1
        offset = numpy.array([center_x, center_y, center_z])

        phi = 0.0
        rotation = numpy.array([phi, theta, psi])

        # from revolutions per second to rad per second
        w = w * 2 * 3.14
        parameters = numpy.array([r, w])

        return traj, offset, rotation, parameters


if __name__ == '__main__':
    try:
        planner = CommandPlanner()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
