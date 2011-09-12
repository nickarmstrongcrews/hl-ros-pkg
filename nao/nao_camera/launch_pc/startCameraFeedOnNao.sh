#!/bin/bash

USERNAME_ON_ROBOT=nao
ROBOT_HOSTNAME=shitbox

ssh $USERNAME_ON_ROBOT@$ROBOT_HOSTNAME "sh ~/ros/nao_stack_hl/nao_camera/launch/setupAndRun.sh"
