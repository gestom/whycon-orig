make neclage
convert -size 2970x2100 xc:white -density 10x10 -units pixelspercentimeter -fill white result.png
w=$(echo "360/$1/2"|bc -l)
n=$(($(./neclage $1|cut -f 2 -d ' '|wc -l)-1))
e=0
for i in $(./neclage $1|cut -f 2 -d ' '|head -n $n);do 
	s=$(echo "obase=2;$i" | bc| xargs printf "%0$1d\n"|sed s/0/AB/g|sed s/1/BA/g|sed s/A/0/g|sed s/B/1/g)
	s=$(echo "obase=2;$i" | bc| xargs printf "%0$1d\n")
	xc=$((150+240*($e/8)))
	yc=$((150+240*($e%8)))
	echo $e $s
	e=$(($e+1))
convert result.png \
-fill white -stroke black -draw "ellipse $xc,$yc 120,120 0,360" \
-fill black -stroke none -draw "ellipse $xc,$yc 100,100 0,360" \
-fill white -stroke none -draw "ellipse $xc,$yc 55,55 0,360" \
result.png

echo $s $e $w
for j in $(seq 0 $(($1-1)));
do
	x1=$(echo "$xc+100*c(-$w*(2*$j+${s:$j:1}*2)/180.0*3.1416)"|bc -l)
	y1=$(echo "$yc+100*s(-$w*(2*$j+${s:$j:1}*2)/180.0*3.1416)"|bc -l)
	x2=$(echo "$xc+100*c(-$w*(2*$j+0*${s:$j:1}+1)/180.0*3.1416)"|bc -l)
	y2=$(echo "$yc+100*s(-$w*(2*$j+0*${s:$j:1}+1)/180.0*3.1416)"|bc -l)
	echo "$xc+100*s($w*(2*$j+${s:$j:1}+1)/180.0*3.1416)"
	echo $x1,$y1,$x2,$x2
convert result.png \
-fill black -stroke none -draw "polygon $xc,$yc $x1,$y1 $x2,$y2" \
result.png
done
convert result.png \
-fill white -stroke none -draw "ellipse $xc,$yc 35,35 0,360" \
result.png
done
convert result.png -density 100x100 result_25.pdf
