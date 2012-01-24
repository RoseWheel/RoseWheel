#!/usr/bin/perl -w

# driveGnuPlots.pl is part of the RoseWheel project.
# Copyright (C) 2011 RoseWheel Team <rosewheel@googlegroups.com>
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.



# Script by Thanassis Tsiodras, Dr.-Ing.

use strict;

sub usage {
    print "Usage: $0 <options>\n";
    print <<OEF;
where mandatory options are (in order):

  NumberOfStreams                        How many streams to plot (windows)
  Stream1_WindowSampleSize               this many samples per window
  <Stream2_WindowSampleSize>             ...per stream2...
...
  <StreamN_WindowSampleSize>             ...per streamN...
  Stream1_YRangeMin Stream1_YRangeMax    Min and Max values for stream 1
  <Stream1_YRangeMin Stream1_YRangeMax>  Min and Max values for stream 2
...
  <StreamN_YRangeMin StreamN_YRangeMax>  Min and Max values for stream N

OEF
    exit(1);
}

sub Arg {
    if ($#ARGV < $_[0]) {
	print "Expected parameter missing...\n\n";
	usage;
    }
    $ARGV[int($_[0])];
}

sub main {
    my $argIdx = 0;
    my $numberOfStreams = Arg($argIdx++);
    print "Will display $numberOfStreams Streams (in $numberOfStreams windows)...\n";
    my @sampleSizes;
    for(my $i=0; $i<$numberOfStreams; $i++) {
	my $samples = Arg($argIdx++);
	push @sampleSizes, $samples;
	print "Stream ".($i+1)." will use a window of $samples samples\n";
    }
    my @ranges;
    for(my $i=0; $i<$numberOfStreams; $i++) {
	my $miny = Arg($argIdx++);
	my $maxy = Arg($argIdx++);
	push @ranges, [ $miny, $maxy ];
	print "Stream ".($i+1)." will use a range of [$miny, $maxy]\n";
    }
    my @gnuplots;
    my @buffers;
    shift @ARGV; # number of streams
    for(my $i=0; $i<$numberOfStreams; $i++) {
	shift @ARGV; # sample size
	shift @ARGV; # miny
	shift @ARGV; # maxy
	local *PIPE;
	open PIPE, "|gnuplot" || die "Can't initialize gnuplot number ".($i+1)."\n";
	select((select(PIPE), $| = 1)[0]);
	push @gnuplots, *PIPE;
	print PIPE "set xtics\n";
	print PIPE "set ytics\n";
	print PIPE "set yrange [".($ranges[$i]->[0]).":".($ranges[$i]->[1])."]\n";
	print PIPE "set style data linespoints\n";
	print PIPE "set grid\n";
	my @data = [];
	push @buffers, @data;
    }
    my $streamIdx = 0;
    select((select(STDOUT), $| = 1)[0]);
    my $xcounter = 0;
    while(<>) {
	chomp;
	my $buf = $buffers[$streamIdx];
	my $pip = $gnuplots[$streamIdx];

	# data buffering (up to stream sample size)
	push @{$buf}, $_;
	#print "stream $streamIdx: ";
	print $pip "set xrange [".($xcounter-$sampleSizes[$streamIdx]).":".($xcounter+1)."]\n";
	print $pip "plot \"-\" notitle\n";
	my $cnt = 0;
	for my $elem (reverse @{$buf}) {
	    #print " ".$elem;
	    print $pip ($xcounter-$cnt)." ".$elem."\n";
	    $cnt += 1;
	}
	#print "\n";
	print $pip "e\n";
	if ($cnt>=$sampleSizes[$streamIdx]) {
	    shift @{$buf};
	}
	$streamIdx++;
	if ($streamIdx == $numberOfStreams) {
	    $streamIdx = 0;
	    $xcounter++;
	}
    }
    for(my $i=0; $i<$numberOfStreams; $i++) {
	my $pip = $gnuplots[$i];
	print $pip "exit;\n";
	close $pip;
    }
}

main;
