


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


# Create a few cards
card1 = Card('Hearts', "Ace")
card2 = Card("Spades", "10")

print(card1)
print(card1.rank, card2.rank)

#Create a new attribute
card1.hidden = True

print(card1.show())
card1.hidden = False
print(card1.show())


# Show their descriptions
# my_card = card1.show()
# your_card = card2.show()
#
# print(my_card, your_card)
# hand = []
# hand.append(my_card)
# hand.append(your_card)
# print(hand)
#


