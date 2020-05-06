

if [ $# -ne 6 ]
then
	echo "useage: $0 kernel_arch target_product qcom_defconfig kernel_defconfig diff kernel_path"
	exit -1
fi

echo ==========================================
echo =======start update diff config ==========
echo ==========================================
#echo $#
#echo $@
config_path="$6/arch/$1/configs"
diff_config=$config_path"/"$5
#diff_config=$config_path"/"$2"_diff"
qcom_config=$config_path"/"$3
defconfig=$config_path"/"$4

sort_diff_config=1

echo config_path=$config_path
echo diff_config=$diff_config
echo qcom_config=$qcom_config
echo defconfig=$defconfig


if [ ! -e $qcom_config ]
then
	echo error: $qcom_config not exist
	exit -1
fi

if [ ! -e $defconfig ]
then
	echo error: $defconfig not exist
	exit -1
fi

if [ -e $diff_config ]
then
	echo mv diff_config to diff_config_old
#if sort the diff config,we use mv here,if not sort,we use cp
	if [ $sort_diff_config -eq 0 ]
	then
		cp $diff_config $diff_config"_old"
	else
		mv $diff_config $diff_config"_old"
	fi
fi

echo creat new diff_config
echo "#diff config for $2" >$diff_config

sort $qcom_config |uniq >temp1
sort $defconfig |uniq >temp2

diff temp1 temp2 >tempdiff
rm temp1
rm temp2

#cat the project defconfig increase item in tempnew
#cat the qcom defconfig reduce item in temp old
echo '' > tempnew
echo '' > tempold
cat tempdiff | while read line
do
	if [[ "$line" == '>'* ]]
	then
		echo ${line:2} >>tempnew
	fi

	if [[ "$line" == '<'* ]]
	then
		echo ${line:2} >>tempold
	fi
done

rm tempdiff

#if a valid item of tempnew also in tempold,
# I think we must save it in tempnew,and delete it in tempold
# what is valid item: it must below 2 format:
# 1. not head of "#",and have a "="
# 2. head of "# ", and end of " is not set"
cat tempold > temp0
cat tempnew | while read line
do
	if [[ ("$line" != '#'*) && ("$line" == *'='*) ]]
	then
#		echo "*=* "$line
		config_name=${line%%=*}
	elif [[ ("$line" == "# CONFIG"*) && ("$line" == *" is not set") ]]
	then
#		echo $line
		config_name=${line:0:0-11}
		config_name=${config_name:2}
	else
		continue
	fi

#we must add "=" or " is not set" here because it possible part of matched
	sed_string=$config_name'='
	sed '/'"$sed_string"'/'d temp0 > temp1
	mv temp1 temp0

	sed_string=$config_name" is not set"
	sed '/'"$sed_string"'/'d temp0 > temp1
	mv temp1 temp0
done
mv temp0 tempold
#in tempold, these item must reduce, we changed it to @item=default_value
#this format will special handing in creat_defconfig.sh
cat tempold |while read line
do
	if [[ ("$line" != '#'*) && ("$line" == *'='*) ]]
	then
#		echo "*=* "$line
		config_name=${line%%=*}

	elif [[ ("$line" == "# CONFIG"*) && ("$line" == *" is not set") ]]
	then
#		echo $line
		config_name=${line:0:0-11}
		config_name=${config_name:2}
	else
		continue
	fi

	echo '-'$config_name >>tempnew
#	echo '@'$config_name"=default_value" >>tempnew
#	echo '#'$config_name"=default_value" >>tempnew
done

#cp $diff_config temp0
if [ $sort_diff_config -eq 0 ]
then
	#first, we delete the same line in tempnew
	cat $diff_config |while read line
	do
		sed '/'"$line"'/'d tempnew > temp0
		mv temp0 tempnew
	done

	cp $diff_config temp0
	cat tempnew |while read line
	do
	#because we must get the item,so must judge all the format
	#and delete the valid item in old diff config
	#	if [[ ("$line" != '#'*) && ("$line" == *'='*) ]]
#		if [[ ("$line" != '#'*) && ("$line" != '@'*) && ("$line" == *'='*) ]]
		if [[ ("$line" != '#'*) && ("$line" != '-'*) && ("$line" == *'='*) ]]
		then
			config_name=${line%%=*}
		elif [[ ("$line" == "# CONFIG"*) && ("$line" == *" is not set") ]]
		then
			config_name=${line:0:0-11}
			config_name=${config_name:2}
#		elif [[ ("$line" == '#'*) && ("$line" == *"=default_value") ]]
#		elif [[ ("$line" == '@'*) && ("$line" == *"=default_value") ]]
		elif [[ "$line" == '-'* ]]
		then
			config_name=${line##*-}
			config_name=${config_name%%=*}
		else
			continue
		fi

		sed_string=$config_name'='
		sed '/'"$sed_string"'/'d temp0 > temp1
		mv temp1 temp0

		sed_string=$config_name" is not set"
		sed '/'"$sed_string"'/'d temp0 > temp1
		mv temp1 temp0
	done
#merge the changed item in to the tail
	cat tempnew >> temp0
	mv temp0 $diff_config
else
sort tempnew >$diff_config
rm tempnew
fi
echo "all ok now, rm $defconfig"
rm $defconfig
