#!/bin/bash

svgs_per_second=2
FPS=30
y_offset=100
fontsize=48

dirs=(
  otf_no_prune
  otf
  labeling
  labeling_model_checked
)

titles=(
  'On-the-fly approach (before pruning)'
  'On-the-fly approach (after pruning)'
  'Offline approach (before model checking)'
  'Offline approach (after model checking)'
)

sections=""
for i in "${!dirs[@]}"; do
  dir=${dirs[$i]}
  title=${titles[$i]}
  path="video/${dir}"
  sections+="file '${PWD}/${path}.mp4'"
  sections+=$'\n'

  ffmpeg -framerate $svgs_per_second -pattern_type glob -height 2160 -i "${path}/*.svg" -c:v libx264 -r $FPS -pix_fmt yuv420p \
    -vf "drawtext=text='${title}':fontcolor=black:fontsize=${fontsize}:x=(w-text_w)/2+40:y=${y_offset}, drawtext=text='k = %{eif\:n\:d\:2}':fontcolor=black:fontsize=${fontsize}:x=w-tw-40:y=${y_offset}, pad=3840:2160:(ow-iw)/2:(oh-ih)/2" \
    "video/$dir.mp4"
done

# concatenate videos
ffmpeg -f concat -safe 0 -i <(echo "${sections}") -c copy "video/all.mp4"
