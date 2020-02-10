#!/bin/sh
for f in $(ls /usr/local/lib | grep opencv | egrep so$); do
    ln -s /usr/local/lib/$f /usr/lib/x86_64-linux-gnu/$f.3.2.0
    ln -s /usr/local/lib/$f /usr/local/lib/$f.3.2.0
    ln -s /usr/local/lib/$f /usr/local/lib/$f.3.2
done