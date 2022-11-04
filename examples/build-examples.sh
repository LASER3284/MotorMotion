for dir in ./*; do
    cd "$dir/cpp"
    chmod +x ./gradlew
    ./gradlew build --refresh-dependencies
done