function hash = release_file_sha256(path)
%RELEASE_FILE_SHA256 SHA-256 without a shell or PowerShell module dependency.
file=fopen(path,'rb');
assert(file>=0,'Release:HashReadFailed','Cannot read: %s',path);
cleanup=onCleanup(@() fclose(file)); %#ok<NASGU>
digest=java.security.MessageDigest.getInstance('SHA-256');
while true
    bytes=fread(file,1048576,'*uint8');
    if isempty(bytes), break; end
    digest.update(typecast(bytes,'int8'));
end
hash=string(sprintf('%02X',typecast(digest.digest(),'uint8')));
end
