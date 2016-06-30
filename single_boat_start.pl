#!/usr/bin/perl
use strict;
use warnings;

my @cmds = ('head /dev/tty_arduino > /dev/null',
'head /dev/tty_ahrs > /dev/null',
'/home/odroid/Documents/boat_agent/custom_controller -i 0 -M /home/odroid/Documents/boat_agent/init.mf --broadcast 192.168.1.255:15000'
);
# cron job apparently requires full paths for everything, including that init.mf file!

my @pids;
for (0..$#cmds) {
	$pids[$_] = fork();
	if ($pids[$_] == 0) {
		system("$cmds[$_]");
		exit;
	}	
}

while (kill 0, @pids) {
	print "...";
	sleep 5;
}

print "FINISHED\n";
