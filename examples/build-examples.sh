for dir in */; do
    echo "Building examples for: $dir"
    cd "$dir/cpp"
    chmod +x ./gradlew
    ./gradlew build --refresh-dependencies
done