set(MKDOCS_DOCS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MKDOCS_SITE_DIR ${CMAKE_CURRENT_BINARY_DIR}/site)

configure_file(mkdocs.yml.in mkdocs.yml)

add_custom_target(multiple_object_tracking_documentation
  ALL
  COMMAND mkdocs build --config-file ${CMAKE_CURRENT_BINARY_DIR}/mkdocs.yml
)
