#!/usr/bin/perl
use strict;
use warnings;

my @cmds = ('./custom_controller -i 0 -M init.mf --multicast 239.255.0.1:4150',
         './custom_controller -i 1 -M init.mf --multicast 239.255.0.1:4150',
         './custom_controller -i 2 -M init.mf --multicast 239.255.0.1:4150',
         './custom_controller -i 3 -M init.mf --multicast 239.255.0.1:4150',
         './custom_controller -i 4 -M init.mf --multicast 239.255.0.1:4150'
	);

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