#!/bin/bash

make neclage > /dev/null # Suppress STDOUT (Standard Output)

# Diplay the help menu
display_help() {
    echo
    echo "Usage: $0 [option...] {help|build|verbose|legacy|slow}" 
    echo
    echo "   -h, --help         Display this help menu"
    echo "   -b, --build        Define a name for the build directory used to generate the images"
    echo "   -v, --verbose      Print a message at each step explaining what is happening"
    echo "   -l, --legacy       Generating the original WhyCon Markers without encoding information"
    echo "   -s, --slow         Processing in a single thread"
    echo "   -d, --distance     Set minimal Hamming distance"
    echo
    exit;
}

# Exit if there are no arguments
if [ $# -eq 0 ]; then
        echo "Try '$0 -h' for more information."; exit;
fi

userFeedback(){ if [ $verbose -eq 1 ]; then echo $@; fi }

# Draw the WhyCode Markers
draw_whyCon_markers(){

    userFeedback "Generate original WhyCon marker"
    # Generate original WhyCon marker
    convert -size 1800x1800 xc:white -density 100x100 -units pixelspercentimeter \
    -fill white -stroke black -draw "ellipse $xc,$yc 899,899 0,360" \
    -fill black -stroke none -draw "ellipse $xc,$yc 700,700 0,360" \
    -fill white -stroke none -draw "ellipse $xc,$yc 420,420 0,360" \
    $( printf "%08d.png" 0 )
    echo "Rendering final image: =>  " $( printf '%08d.bmp' $2 )
}

# Draw the WhyCode Markers
draw_whyCode_markers(){

    userFeedback "Generating WhyCode Canvas for Id $2 (encoding $1)"
    # Generate original WhyCon marker
    convert -size 1800x1800 xc:white -density 100x100 -units pixelspercentimeter \
    -fill white -stroke black -draw "ellipse $xc,$yc 899,899 0,360" \
    -fill black -stroke none -draw "ellipse $xc,$yc 700,700 0,360" \
    -fill white -stroke none -draw "ellipse $xc,$yc 420,420 0,360" \
    $buildDir/buildFile-$1.png

    userFeedback "Converting ID to Binary"
    # Convert lowest bit shift to binary
    s=$(echo "obase=2;$1" | bc| xargs printf "%0$3d\n")

    # For each encoding bit
    for j in $(seq 0 $(($3-1)));do

        # Calculate the pixel positions of each segment
        x1=$(echo "$xc+650*c(-$w*(2*$j+${s:$j:1}*2)/180.0*$pi)"|bc -l)
        y1=$(echo "$yc+650*s(-$w*(2*$j+${s:$j:1}*2)/180.0*$pi)"|bc -l)
        x2=$(echo "$xc+650*c(-$w*(2*$j+0*${s:$j:1}+1)/180.0*$pi)"|bc -l)
        y2=$(echo "$yc+650*s(-$w*(2*$j+0*${s:$j:1}+1)/180.0*$pi)"|bc -l)
        userFeedback "Drawing Segment Size: $x1 $y1 $x2 $y2"
        # Draw each of the segments onto the original WhyCon marker
        convert $buildDir/buildFile-$1.png -fill black -stroke none -draw "polygon $xc,$yc $x1,$y1 $x2,$y2" $buildDir/buildFile-$1.png
    done

    # Draw a final white circle in the centre to complete the marker
    echo "Rendering final image: $2 (encoding $1)  =>  " $( printf '%08d.bmp' $2 )
    convert $buildDir/buildFile-$1.png -flatten -fill white -stroke none -draw "ellipse $xc,$yc 240,240 0,360" $( printf "%08d.png" $2 )
    
    # User feedback - encoded_id - file
    # echo $1     $( printf '%08d.bmp' $2 )
}

# ============================================================================
# ========================= WhyCode Marker Generator =========================
# ============================================================================

# Define default arguments
buildDir=".WhyConBuildDirectory"
verbose=0
legacy=0
multithread=0
validateBuildDir=0
teethCount=""
hamming=1

while : 
do
    case "$1" in
        -h | --help) display_help ;;
        -b | --build) buildDir="$2"; shift 2; ;;
        -d | --distance) hamming="$2"; shift 2; ;;
        -v | --verbose) verbose=1; shift 1;;
        -l | --legacy) legacy=1; shift 1;;
        -s | --slow) multithread=1; shift 1;;
        --) shift; break ;; # End of all options
        -*) echo ; echo "Error: Unknown option: $1"; display_help; exit ;;
        *) if [ $# -eq 0 ]; then break; elif [[ $1 =~ ^[0-9]+$ ]]; then teethCount=$1; shift 1; fi
    esac
done

# Calculate Pi to 50 decimal places
pi=$(echo "scale=50; 4*a(1)" | bc -l)

# Centre of the 1800x1800 image
xc=900
yc=900

# Create the build directory
if [ ! -d "$buildDir" ]; then
    userFeedback "Creating directory $buildDir"
    mkdir $buildDir
else
    echo ; echo "Error: $buildDir already exsists. Consider changing it"
    display_help
fi

# Check if legacy option selected
if [ $legacy -eq 1 ]; then 
    # Draw the original WhyCon markers
    draw_whyCon_markers
else

if [ -z $teethCount ]; then
    echo ; 
    echo "Error: Number of encoding bits not specified."; 
    display_help;
fi

# The encodings index (not its encoded value)
index=0

# The number of possible IDs available
n=$(($(./neclage $teethCount $hamming|cut -f 2 -d ' '|wc -l)))

# the width of an encoding
w=$(echo "360/$teethCount/2"|bc -l)

    # for each ID
    for id in $(./neclage $teethCount $hamming|cut -f 2 -d ' '|head -n $n);do 

        # Used to keep track of marker index in order to name the output files
        index=$(( $index+1 ))

        # If processing on all cores
        if [ $multithread -eq 1 ]; then
                draw_whyCode_markers $id $index $teethCount
            else
                draw_whyCode_markers $id $index $teethCount &
        fi
    done
fi

# wait for all processes to finish
wait

# remove unwanted 
if [ -d "$buildDir" ]; then
    userFeedback "Cleaning up. $buildDir directory"
    rm -r $buildDir
fi
