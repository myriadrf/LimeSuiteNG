base_hash=$1
if [ "$base_hash" == "" ]; then
    base_hash=$(git rev-list --max-parents=0 HEAD)
fi

source_patch=$(git-clang-format --diff --commit "$base_hash")
echo $source_patch
[ "$source_patch" = "no modified files to format" ] && exit 0
[ "$source_patch" = "clang-format did not modify any files" ] && exit 0

printf "\033[1mYou have introduced coding style breakages, suggested changes:\n\n"

echo "$source_patch" | colordiff
exit 1
