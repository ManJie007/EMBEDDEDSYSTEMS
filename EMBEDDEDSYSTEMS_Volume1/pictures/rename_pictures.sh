#!/bin/bash

# iterate all image file
for file in *.{jpg,jpeg,png}; do
  #check file exist
  if [ -e "$file" ]; then
    #change file name
    new_file="${file// /_}"
    #if name is different, change the file name
    if [ "$file" != "$new_file" ]; then
      mv "$file" "$new_file"
      echo "Renamed: \"$file\" to \"$new_file\""
    fi
  fi
done

echo "change file name done!!!"

