function root = project_codes_root()
%PROJECT_CODES_ROOT Stable Codes directory, independent of caller location.
root=string(fileparts(fileparts(mfilename('fullpath'))));
end
