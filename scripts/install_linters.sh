#!/bin/bash

pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_copyright-0.8.1&subdirectory=ament_copyright"
pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_cppcheck-0.8.1&subdirectory=ament_cppcheck"
pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_cpplint-0.8.1&subdirectory=ament_cpplint"
pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_flake8-0.8.1&subdirectory=ament_flake8"
pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_lint-0.8.1&subdirectory=ament_lint"
pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_mypy-0.8.1&subdirectory=ament_mypy"
pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_pep257-0.8.1&subdirectory=ament_pep257"
pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_uncrustify-0.8.1&subdirectory=ament_uncrustify"
pip3 install "git+https://github.com/ament/ament_lint.git@0.8.1#egg=ament_xmllint-0.8.1&subdirectory=ament_xmllint"

apt-get install --no-install-recommends --yes \
  cppcheck \
  uncrustify