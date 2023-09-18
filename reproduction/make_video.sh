#!/bin/bash

svgs_per_second=2
FPS=30
y_offset=100
fontsize=48

video_dir="video"
output_name="animation_yield.mp4"

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

ffmpeg -framerate 0.2 -height 2160 -i "${video_dir}/title.svg" -c:v libx264 -r 30 -pix_fmt yuv420p "${video_dir}/title.mp4"
sections="file '${PWD}/${video_dir}/title.mp4'"
sections+=$'\n'

for i in "${!dirs[@]}"; do
  dir=${dirs[$i]}
  title=${titles[$i]}
  path="${video_dir}/${dir}"
  sections+="file '${PWD}/${path}.mp4'"
  sections+=$'\n'

  ffmpeg -framerate $svgs_per_second -pattern_type glob -height 2160 -i "${path}/*.svg" -c:v libx264 -r $FPS -pix_fmt yuv420p \
    -vf "drawtext=text='${title}':fontcolor=black:fontsize=${fontsize}:x=(w-text_w)/2+40:y=${y_offset}, drawtext=text='k = %{eif\:n\:d\:2}':fontcolor=black:fontsize=${fontsize}:x=w-tw-40:y=${y_offset}, pad=3840:2160:(ow-iw)/2:(oh-ih)/2" \
    "${video_dir}/${dir}.mp4"
done

# concatenate videos
ffmpeg -f concat -safe 0 -i <(echo "${sections}") -c copy "${output_name}"

echo "Video saved as ${output_name}"
