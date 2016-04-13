#!/bin/sh


cat $1 | sed -e "s/iris[0-9]/$2/g" > $3

echo $3