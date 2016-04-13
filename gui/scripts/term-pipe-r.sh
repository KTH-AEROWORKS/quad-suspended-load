#!/bin/bash

source ~/.bashrc

pipe=$1                     # the pipe name is the first argument
rm $pipe
trap 'rm -f "$pipe"' EXIT     # ignore exit and delete messages until the end

set +x
if [ ! -p $pipe ] 
then   # if the pipe doesn't exist, create it
    mkfifo $pipe
fi

while true                  # cycle eternally..
do
    if read line < $pipe; then
        if [[ "$line" == 'close the term-pipe pipe' ]]; then
            break
            # if the pipe closing message is received, break the while cycle
        fi
        
        echo                # a line break should be used because of the prompt 
        echo "$(tput bold; tput setaf 0; tput setab 7)$line$(tput sgr 0)"
        eval $line  &        # run the line: as this script should be started
        line=""
    fi                          # in the target terminal, 
done                            # the line will be run there.

echo "<pipe closing message>"   # custom message on the end of the script

#So say you want /dev/tty3 to receive commands: just go there, do
#./term-pipe-r.sh tty3pipe &     # $1 will be tty3pipe (in a new process)
#And to send commands, from any terminal (even from itself):
#echo "command" > tty3pipe
#or to run a file there:
#cat some-script.sh > tty3pipe

