if [ $# -eq 0 ]
  then
    echo "Codename argument not supplied"
    exit
fi

changelog_list=`ls -1 ./*changelog`

for i in ${changelog_list}
do
	dch --changelog "$i" --nomultimaint --distribution $1 --local ~$1.myriadrf ""
	dch --changelog "$i" --nomultimaint --release ""
	if [ $? -ne 0 ]
	  then exit
	fi
done
