function verify_release_deployment()
%VERIFY_RELEASE_DEPLOYMENT Verify the release's actual runtime configuration.
% Separate from historical training/evaluation provenance. No hash waiver.
root=string(project_codes_root());
manifest=readtable(fullfile(root,'release_runtime_checksums.csv'),'TextType','string');
for k=1:height(manifest)
    path=fullfile(root,manifest.File(k));
    assert(isfile(path),'Release:MissingDependency','Missing dependency: %s',path);
    actual=release_file_sha256(path);
    assert(actual==manifest.SHA256(k),'Release:DependencyChanged', ...
        'Release dependency changed: %s\nExpected: %s\nActual: %s', ...
        path,manifest.SHA256(k),actual);
end
end
