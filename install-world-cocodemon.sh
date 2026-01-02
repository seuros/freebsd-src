#!/bin/sh
#
# Install world on cocodemon from NFS-mounted source
# Run this from cocodemon: cd /mnt/src && sudo ./install-world-cocodemon.sh
#

set -e  # Exit on error
set -x  # Show commands

echo "=== FreeBSD World Installation Script ==="
echo "Starting installworld from NFS..."
echo ""

# Set environment variables
export MAKEOBJDIRPREFIX=/mnt/obj
export COMPILER_TYPE=clang
export OBJTOP=/mnt/obj/usr/src/amd64.amd64

# Verify we're in the right place
if [ ! -f Makefile ]; then
    echo "ERROR: Must run from /mnt/src directory!"
    exit 1
fi

# Verify NFS mounts exist
if [ ! -d /mnt/obj/usr/src/amd64.amd64 ]; then
    echo "ERROR: /mnt/obj not properly mounted!"
    exit 1
fi

echo "Environment:"
echo "  MAKEOBJDIRPREFIX: $MAKEOBJDIRPREFIX"
echo "  COMPILER_TYPE: $COMPILER_TYPE"
echo "  Working directory: $(pwd)"
echo ""

# Run installworld
# Use same WITHOUT_* options that were used during buildworld
echo "=== Running installworld (this will take 10-15 minutes) ==="
make DESTDIR=/ \
  WITHOUT_NLS=yes WITHOUT_NLS_CATALOGS=yes WITHOUT_LOCALES=yes \
  WITHOUT_ICONV=yes WITHOUT_INET6=yes WITHOUT_INET6_SUPPORT=yes \
  WITHOUT_BLUETOOTH=yes WITHOUT_WIRELESS=yes WITHOUT_WIRELESS_SUPPORT=yes \
  WITHOUT_MAIL=yes WITHOUT_SENDMAIL=yes WITHOUT_MAILWRAPPER=yes \
  WITHOUT_FTP=yes WITHOUT_TELNET=yes WITHOUT_ROUTED=yes WITHOUT_NIS=yes \
  WITHOUT_RBOOTD=yes WITHOUT_LPR=yes WITHOUT_GAMES=yes WITHOUT_DICT=yes \
  WITHOUT_CALENDAR=yes WITHOUT_FINGER=yes WITHOUT_TALK=yes WITHOUT_AT=yes \
  WITHOUT_EXAMPLES=yes WITHOUT_SHAREDOCS=yes WITHOUT_HTML=yes \
  WITHOUT_BHYVE=yes WITHOUT_HAST=yes WITHOUT_TESTS=yes \
  WITHOUT_TESTS_SUPPORT=yes WITHOUT_KERBEROS=yes \
  WITHOUT_KERBEROS_SUPPORT=yes WITHOUT_BSDINSTALL=yes \
  WITHOUT_MAN=yes \
  installworld

echo ""
echo "=== Installworld completed successfully! ==="
echo ""
echo "IMPORTANT: You must REBOOT now for changes to take effect:"
echo "  sudo shutdown -r now"
echo ""
