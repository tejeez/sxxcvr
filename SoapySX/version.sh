#!/bin/sh
# Print a source file with git tag and commit number.
# $1 is GIT_EXECUTABLE from CMakeLists.txt.

# For some reason it sometimes shows dirty if changes have just been committed
# but stops doing it after running git status.
# This may not be the right way to fix it but it seems to help.
"$1" status > /dev/null

set -e

echo -n 'const char *SoapySX_tag = "'
"$1" describe --tags --always --dirty --broken | tr -d '\n'
echo '";'

echo -n 'const char *SoapySX_commit = "'
"$1" rev-parse HEAD | tr -d '\n'
"$1" diff-index --quiet HEAD -- || echo -n -dirty
echo '";'
