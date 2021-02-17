#!/bin/sh

which sed || (echo "sed unavailable" 1>&2; exit 1)

echo "Creating obj directories..."

test -d lencod/obj && rm -rf lencod/obj ; mkdir lencod/obj
test -d ldecod/obj && rm -rf ldecod/obj ; mkdir ldecod/obj

rm -f lencod/dependencies; touch lencod/dependencies
rm -f ldecod/dependencies; touch ldecod/dependencies

echo "Removing DOS LF chars..."
for f in l[ed][ne]cod/[si][rn]c/*.[ch]
do
   sed -e "s///" < $f >$f.tmp && mv $f.tmp $f
done

for f in l[ed][ne]cod/Makefile
do
   sed -e "s///" < $f >$f.tmp && mv $f.tmp $f
done

echo "Done."

