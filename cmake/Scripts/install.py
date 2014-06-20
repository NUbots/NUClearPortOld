#!/usr/bin/python

import argparse
import glob
from subprocess import call

parser = argparse.ArgumentParser()

parser.add_argument('--robot_ip')
parser.add_argument('--config')
parser.add_argument('--username')

args = parser.parse_args()

robot_ip = args.robot_ip
config = args.config
username = args.username if args.username else 'darwin'

if not robot_ip:
    raise 'Robot IP not given (robot=ip)'

target_dir = '{0}@{1}:/home/{0}/'.format(username, robot_ip)

# Copy the binaries over
files = glob.glob('bin/*')
call(['rsync', '-avzP', '--checksum', '-e ssh'] + files + [target_dir])


# Overwrite configuration files
if config in ['overwrite', 'o']:
    print 'Updating configuration files'
    call(['rsync', '-avzP', '--checksum', '-e ssh', 'config', target_dir])

# Update configuration files
elif config in ['update', 'u']:
    print 'Adding new configuration files only'
    call(['rsync', '-avzuP', '--checksum', '-e ssh', 'config', target_dir])

# Add new configuration files
elif not config or config in ['new', 'n']:
    print 'Adding new configuration files only'
    call(['rsync', '-avzP', '--checksum', '--ignore-existing', '-e ssh', 'config', target_dir])

# Ignore configuration files
elif config in ['ignore', 'i']:
    print "Ignoring configuration changes"

# Bad configuration parameters
else:
    print 'The configuration parameters must be either one of update, new or ignore (u, n or i)'
