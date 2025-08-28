

# Define a simple class to represent a playing card
class Card:
        """
        Create a card with a suit (e.g., "Hearts") and rank (e.g., "Ace").
        """

        def __init__(self, suit, rank):
                self.suit = suit
                self.rank = rank


        def show(self):
                """
                Print a description of the card.
                """
                # print(f"{self.rank} of {self.suit}")
                # return f"{self.rank} of {self.suit}"

                """" Check if the instance has an attribute"""
                if hasattr(self, 'hidden'):
                        if self.hidden == False:
                                return f"{self.rank} of {self.suit}"


                return "Card cannot be shown"
#__________________________________________________________#

#Define a new class that inherits from the card class
class SpecialCard(Card):
    def __init__(self, suit, rank, power):
        # Call the parent constructor using super()
        super().__init__(suit, rank)

        self.power = power


    #Method of the child class
    def show_power(self):
        print(f"The card power is {self.power}")


# Create a few cards

card1 = Card("Hearts", "Ace")
card2 = Card("Spades", "10")

card3 = SpecialCard("Clubs", "King", 10) # Create a card from the SpecialCard class

#Create a new attribute
card1.hidden = True
card3.hidden = False

# Show their descriptions
print(card1.show())     # Ace of Hearts
print(card2.show())     # 10 of Spades

print(card3.show())
card3.show_power()