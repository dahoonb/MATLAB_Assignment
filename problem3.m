randomNumbers = zeros(1,10);

for k = 1:10
    randomNumbers(k) = rand;
end

plot(1:10, randomNumbers)

xlabel("Index")
ylabel("Random Number")