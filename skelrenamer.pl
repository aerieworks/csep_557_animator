#!/usr/bin/perl -w
# File: skelrenamer.pl
# Renames a file from *.skel to *

# Undefine the input record specifier, $/, which is normally "\n", so that
# the whole file is read as one long record, newlines included.
undef $/;

# process each file on the command line

foreach $file (@ARGV) {
	if (!open(INPUT,"<$file")) {
		print STDERR "Can't open input file $file\n";
		next;
	}

	$noskel = $file;
	$noskel =~ s/\.skel//;
	if (!open(OUTPUT, ">$noskel")) {
		print STDERR "Can't open output file $noskel\n";
		next;
	}
	print OUTPUT <INPUT>;

	close INPUT;
	close OUTPUT;
}
