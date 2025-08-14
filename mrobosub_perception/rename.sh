#!/bin/bash
for f in 08142025_480_640_*.png; do
	mv -- "$f" "${f#08142025_480_640_}"
done
