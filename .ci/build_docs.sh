#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"
. devel/setup.bash

ADA_DEMOS_DIR="${HOME}/workspace/src/ada_demos"

# For branch builds, Travis only clones that branch with a fixed depth of 50
# commits. This means that the clone knows nothing about other Git branches or
# tags. We fix this by deleting and re-cloning the full repository.
rm -rf ${ADA_DEMOS_DIR}
git clone "https://github.com/${GITHUB_REPOSITORY}.git" ${ADA_DEMOS_DIR}

# Organize into "gh-pages" directory
mkdir -p ${GITHUB_WORKSPACE}/gh-pages

# Initialize list of API versions
cat <<EOF > ${GITHUB_WORKSPACE}/gh-pages/README.md
## API Documentation

EOF

mkdir build_docs
cd build_docs

while read version; do
  # Add entry to list of API versions
  echo "* [${version}](https://personalrobotics.github.io/ada_demos/${version}/)" >> ${GITHUB_WORKSPACE}/gh-pages/README.md

  # Build documentation
  git -C ${ADA_DEMOS_DIR} checkout ${version}
  rm -rf *
  cmake -DDOWNLOAD_TAGFILES=ON ${ADA_DEMOS_DIR}
  make docs
  mv doxygen ${GITHUB_WORKSPACE}/gh-pages/${version}
done < ${GITHUB_WORKSPACE}/.ci/docs_versions.txt
