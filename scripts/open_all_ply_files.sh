#!/bin/bash

HELP="Usage:
	./open_all_ply_files.sh [folder with ply files]"

if test "$#" -ne 1; then
    echo "Illegal number of parameters!"
    echo $HELP
    exit 1
fi

# Add header
HEADER="<!DOCTYPE MeshLabDocument>\n<MeshLabProject>\n <MeshGroup>\n"


# Read all the ply file paths
INPUT_FOLDER=$1
PLY_FILES=( "$INPUT_FOLDER"/*.ply )

# Add the mesh items
MLP_FILE="$HEADER"
for (( i = 0 ; i < ${#PLY_FILES[@]} ; i++ )) do
	PLY_FILE="${PLY_FILES[$i]}"

	# Split the paths into path file and filename (no .ply)
	COMPLETE_PATH=$PLY_FILE
	PATH_TO_FILE=${COMPLETE_PATH%/*}
	FILE=${COMPLETE_PATH##*/}
	FILE_NAME=${FILE%.*}


	echo -e "${blue}${i} -> PLY File: $PLY_FILE${nocolor}"

	MLP_ITEM="  <MLMesh label=\"$FILE_NAME\" filename=\"$COMPLETE_PATH\">
   <MLMatrix44>
1 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1
</MLMatrix44>
  </MLMesh>\n"
	MLP_FILE="$MLP_FILE$MLP_ITEM"
done

# Add footer
MLP_FILE="$MLP_FILE </MeshGroup>\n <RasterGroup/>\n</MeshLabProject>"

# Create file
rm -rf "open_all.mlp"
echo -e "$MLP_FILE" >> "open_all.mlp"

# Open meshlab
meshlab "open_all.mlp"
