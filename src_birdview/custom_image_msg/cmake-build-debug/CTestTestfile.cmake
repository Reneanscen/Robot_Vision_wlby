# CMake generated Testfile for 
# Source directory: /home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg
# Build directory: /home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(cppcheck "/usr/bin/python3.8" "-u" "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/cppcheck.xunit.xml" "--package-name" "custom_image_msg" "--output-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/ament_cppcheck/cppcheck.txt" "--command" "/home/xu/ros2_humble/install/ament_cppcheck/bin/ament_cppcheck" "--xunit-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/cppcheck.xunit.xml" "--include_dirs" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/rosidl_generator_c" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/rosidl_typesupport_fastrtps_c" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/rosidl_generator_cpp" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/rosidl_typesupport_fastrtps_cpp" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/rosidl_typesupport_introspection_c" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/rosidl_typesupport_introspection_cpp" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/rosidl_generator_c" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/rosidl_generator_py")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg" _BACKTRACE_TRIPLES "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/xu/ros2_humble/install/ament_cmake_cppcheck/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/home/xu/ros2_humble/install/ament_cmake_cppcheck/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;83;ament_cppcheck;/home/xu/ros2_humble/install/ament_cmake_cppcheck/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;36;ament_package;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;0;")
add_test(flake8 "/usr/bin/python3.8" "-u" "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/flake8.xunit.xml" "--package-name" "custom_image_msg" "--output-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/ament_flake8/flake8.txt" "--command" "/home/xu/ros2_humble/install/ament_flake8/bin/ament_flake8" "--xunit-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/flake8.xunit.xml")
set_tests_properties(flake8 PROPERTIES  LABELS "flake8;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg" _BACKTRACE_TRIPLES "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/xu/ros2_humble/install/ament_cmake_flake8/share/ament_cmake_flake8/cmake/ament_flake8.cmake;63;ament_add_test;/home/xu/ros2_humble/install/ament_cmake_flake8/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;18;ament_flake8;/home/xu/ros2_humble/install/ament_cmake_flake8/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;36;ament_package;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3.8" "-u" "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/lint_cmake.xunit.xml" "--package-name" "custom_image_msg" "--output-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/ament_lint_cmake/lint_cmake.txt" "--command" "/home/xu/ros2_humble/install/ament_lint_cmake/bin/ament_lint_cmake" "--xunit-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg" _BACKTRACE_TRIPLES "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/xu/ros2_humble/install/ament_cmake_lint_cmake/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/home/xu/ros2_humble/install/ament_cmake_lint_cmake/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/home/xu/ros2_humble/install/ament_cmake_lint_cmake/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;36;ament_package;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;0;")
add_test(pep257 "/usr/bin/python3.8" "-u" "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/pep257.xunit.xml" "--package-name" "custom_image_msg" "--output-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/ament_pep257/pep257.txt" "--command" "/home/xu/ros2_humble/install/ament_pep257/bin/ament_pep257" "--xunit-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/pep257.xunit.xml")
set_tests_properties(pep257 PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg" _BACKTRACE_TRIPLES "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/xu/ros2_humble/install/ament_cmake_pep257/share/ament_cmake_pep257/cmake/ament_pep257.cmake;41;ament_add_test;/home/xu/ros2_humble/install/ament_cmake_pep257/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;18;ament_pep257;/home/xu/ros2_humble/install/ament_cmake_pep257/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;36;ament_package;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3.8" "-u" "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/uncrustify.xunit.xml" "--package-name" "custom_image_msg" "--output-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/ament_uncrustify/uncrustify.txt" "--command" "/home/xu/ros2_humble/install/ament_uncrustify/bin/ament_uncrustify" "--xunit-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg" _BACKTRACE_TRIPLES "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/xu/ros2_humble/install/ament_cmake_uncrustify/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;70;ament_add_test;/home/xu/ros2_humble/install/ament_cmake_uncrustify/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;34;ament_uncrustify;/home/xu/ros2_humble/install/ament_cmake_uncrustify/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;36;ament_package;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3.8" "-u" "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/xmllint.xunit.xml" "--package-name" "custom_image_msg" "--output-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/ament_xmllint/xmllint.txt" "--command" "/home/xu/ros2_humble/install/ament_xmllint/bin/ament_xmllint" "--xunit-file" "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/cmake-build-debug/test_results/custom_image_msg/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg" _BACKTRACE_TRIPLES "/home/xu/ros2_humble/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/xu/ros2_humble/install/ament_cmake_xmllint/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/home/xu/ros2_humble/install/ament_cmake_xmllint/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/home/xu/ros2_humble/install/ament_cmake_xmllint/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/xu/ros2_humble/install/ament_lint_auto/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/xu/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;36;ament_package;/home/xu/ZME/zme_BirdView/bird_vision/src/custom_image_msg/CMakeLists.txt;0;")
subdirs("custom_image_msg__py")
