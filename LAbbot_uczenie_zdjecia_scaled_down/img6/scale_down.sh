#!/bin/bash

echo 'scaling down images..'
FILES=$(pwd)
for f in *.jpg
do
	echo "ebe ebe $f" 
	convert $f -resize 455x256\! $f
done
echo "done"

