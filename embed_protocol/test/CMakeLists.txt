include_directories(
  ${CMAKE_SOURCE_DIR}/include
)

set(dependencies
  toml
)

ament_add_gtest(
  embed_protocol_test embed_protocol_test/embed_protocol_test.cpp
  TIMEOUT 20
)
ament_target_dependencies(embed_protocol_test ${dependencies})

target_compile_definitions(embed_protocol_test PRIVATE
  "PASER_PATH=\"${CMAKE_SOURCE_DIR}/test/embed_protocol_test/parser\""
  "COMMON_PROTOCOL_TEST=true"
)
