# Dissect the version specified in PROJECT_VERSION, placing the major,
# minor, revision and candidate components in PROJECT_VERSION_MAJOR, etc.
# _prefix: The prefix string for the version variable names.

# Filter a list to remove all strings matching the regex in _pattern. The
# output is placed in the variable pointed at by _output.
macro(FILTER_LIST _list _pattern _output)
    set(${_output})
    foreach(_item ${_list})
        if("${_item}" MATCHES ${_pattern})
            set(${_output} ${${_output}} ${_item})
        endif("${_item}" MATCHES ${_pattern})
    endforeach(_item)
endmacro(FILTER_LIST)

